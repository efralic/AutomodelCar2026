#include <iostream>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int16.h>
#include <std_msgs/UInt8.h>
#include <math.h>
#include "object_detection/points_objects.h"
#include <string>
#include <vector>
#include <chrono>
#include <thread>
#include <sensor_msgs/Range.h>   // ← NUEVO: para ultrasonidos

// ── Estados ──────────────────────────────────────────────────────────────────
//
//  LANE_DRIVING     : avanza por el carril siguiendo la línea
//  ALIGN_TO_SPACE   : avanza un poco más para alinear el eje trasero al cajón
//  PARKING_IN       : gira derecha y retrocede hasta que ultrasónico trasero
//                     detecta la banqueta
//  STOP             : estacionado, velocidad 0
//
// Flujo:
//   LANE_DRIVING ──(ultrasónico derecho detecta espacio vacío)──▶ ALIGN_TO_SPACE
//   ALIGN_TO_SPACE ──(timer ALIGN_TIME_MS)──────────────────────▶ PARKING_IN
//   PARKING_IN ──(ultrasónico trasero < WALL_THRESHOLD)─────────▶ STOP
//
// Dimensiones del cajón (reglamento TMR 2026):
//   Ancho  : 40 cm  (≈ 2× ancho del AutoModelCar)
//   Fondo  : 60 cm máximo
// ─────────────────────────────────────────────────────────────────────────────

int LANE_DRIVING   = 0;
int ALIGN_TO_SPACE = 1;
int PARKING_IN     = 2;
int STOP           = 3;

static std::map<int, std::string> task_names{
    {0, "Lane driving"},
    {1, "Align to parking space"},
    {2, "Parking in"},
    {3, "Parked - stop"}
};

// ── Umbrales ultrasonido ──────────────────────────────────────────────────────
// SPACE_DETECT_THRESHOLD: si el ultrasonido DERECHO lee MÁS que este valor
//   significa que no hay coche al lado → cajón vacío detectado.
//   Los coches vecinos deberían estar a ~5-10 cm del tuyo en la pista,
//   el cajón vacío estará a mucho más. Ajustar según pista real.
static const float SPACE_DETECT_THRESHOLD = 0.25f; // metros

// WALL_THRESHOLD: el ultrasonido TRASERO para de retroceder cuando
//   detecta la banqueta a menos de esta distancia.
//   El cajón tiene 60 cm de fondo, el carro mide ~20 cm → margen ~10 cm.
static const float WALL_THRESHOLD         = 0.10f; // metros (10 cm)

// ALIGN_TIME_MS: tiempo extra que avanza tras detectar el espacio para
//   que el eje trasero quede alineado con el cajón antes de girar.
//   Ajustar en pista real.
static const long  ALIGN_TIME_MS          = 600;   // milisegundos


class Task {
public:
    std::string name;
    int ID;
    Task(int task_identifier) {
        this->ID   = task_identifier;
        this->name = task_names[this->ID];
    }
};


class Master {
private:
    ros::NodeHandle nh_;

    // Subscribers
    ros::Subscriber distance_center_sub;
    ros::Subscriber ultrasonic_right_sub;
    ros::Subscriber ultrasonic_back_sub;

    // Publishers
    ros::Publisher angle_pub;
    ros::Publisher speed_pub;

    std_msgs::Int16 angle_message;
    std_msgs::Int16 speed_message;

    // Lane following
    int   dist_now;
    int   angle_last;
    float kp_angle;
    float kd_angle;
    int   u_angle;
    int   angle_pd;
    int   speed_pid;
    float kp_speed;
    int   u_speed;

    // Task management
    std::vector<Task> task_pile;

    // Ultrasonidos
    float ultra_right;
    float ultra_back;
    bool  space_detected_once;

    // Timer para ALIGN_TO_SPACE
    std::chrono::steady_clock::time_point align_start;

public:
    Master(Task task) {
        distance_center_sub  = nh_.subscribe("/distance_center_line", 1,
                                              &Master::dist_center_clbk, this);
        ultrasonic_right_sub = nh_.subscribe("/sensors/ultrasonic/right", 1,
                                              &Master::ultra_right_clbk, this);
        ultrasonic_back_sub  = nh_.subscribe("/sensors/ultrasonic/back",  1,
                                              &Master::ultra_back_clbk,  this);

        angle_pub = nh_.advertise<std_msgs::Int16>(
                        "/AutoModelMini/manual_control/steering", 1000);
        speed_pub = nh_.advertise<std_msgs::Int16>(
                        "/AutoModelMini/manual_control/speed",    1000);

        dist_now  = angle_last = 0;
        kp_angle  = 0.825f;
        kd_angle  = 0.0297f;
        u_angle   = angle_pd = speed_pid = u_speed = 0;
        kp_speed  = 1.2645f;

        ultra_right         = 9.9f;
        ultra_back          = 9.9f;
        space_detected_once = false;

        this->add_task(task);
        ROS_INFO("Master_parking listo - buscando espacio de estacionamiento");
    }

    // ── Callbacks ─────────────────────────────────────────────────────────────
    void ultra_right_clbk(const sensor_msgs::Range::ConstPtr& msg) {
        ultra_right = msg->range;
    }

    void ultra_back_clbk(const sensor_msgs::Range::ConstPtr& msg) {
        ultra_back = msg->range;
    }

    void dist_center_clbk(const std_msgs::Int16& dis_now_center) {
        dist_now = static_cast<int>(dis_now_center.data);
        this->run();
        this->publish_policies();
    }

    // ── Task management ───────────────────────────────────────────────────────
    void add_task(Task task)  { task_pile.push_back(task); }
    void remove_task() {
        if (task_pile.size() > 1) task_pile.pop_back();
    }
    Task get_current_task() { return task_pile.back(); }

    // ── task_assigner: detecta el espacio con ultrasonido derecho ─────────────
    void task_assigner() {
        if (get_current_task().ID != LANE_DRIVING) return;
        if (space_detected_once) return;

        // Si el ultrasonido derecho ve más lejos que el umbral → cajón vacío
        if (ultra_right > SPACE_DETECT_THRESHOLD) {
            space_detected_once = true;
            align_start = std::chrono::steady_clock::now();
            ROS_INFO("Cajón detectado (derecho=%.2f m) - alineando eje trasero...",
                     ultra_right);
            add_task(Task(ALIGN_TO_SPACE));
        }
    }

    // ── task_solver ───────────────────────────────────────────────────────────
    void task_solver() {
        Task current = get_current_task();
        ROS_INFO_STREAM_THROTTLE(0.5,
            "[Estado]: " << current.name
            << " | derecho=" << ultra_right << "m"
            << " | trasero=" << ultra_back  << "m");

        // LANE DRIVING: sigue la línea mientras busca el cajón
        if (current.ID == LANE_DRIVING) {
            on_lane();
        }

        // ALIGN TO SPACE: avanza ALIGN_TIME_MS ms extra para que
        // el eje trasero del carro quede frente al cajón
        else if (current.ID == ALIGN_TO_SPACE) {
            on_lane();
            long elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                               std::chrono::steady_clock::now() - align_start).count();
            if (elapsed >= ALIGN_TIME_MS) {
                ROS_INFO("Alineado - iniciando entrada al cajón");
                remove_task();
                add_task(Task(PARKING_IN));
            }
        }

        // PARKING IN: retrocede girando a la derecha
        // El carro entra al cajón hasta que el ultrasonido trasero
        // detecta la banqueta (< WALL_THRESHOLD)
        //
        // ⚠ IMPORTANTE - dirección en reversa:
        //   angle_pd > 90 = giro derecha avanzando = giro IZQUIERDA retrocediendo
        //   angle_pd < 90 = giro izquierda avanzando = giro DERECHA retrocediendo
        //   Como queremos entrar al cajón que está a la DERECHA retrocediendo,
        //   usamos angle_pd = 45 (izquierda hacia adelante = derecha en reversa)
        //   Ajustar según el comportamiento real del servo en reversa.
        else if (current.ID == PARKING_IN) {
            angle_pd  = 45;    // giro derecha en reversa (ajustar en pista)
            speed_pid = 250;   // reversa — si el carro avanza en lugar de
                               // retroceder, cambia el signo en motor_driver.py

            if (ultra_back < WALL_THRESHOLD) {
                ROS_INFO("Banqueta detectada (%.2f m) - estacionamiento completo",
                         ultra_back);
                speed_pid = 0;
                angle_pd  = 90;
                remove_task();
                add_task(Task(STOP));
            }
        }

        // STOP: estacionado
        else if (current.ID == STOP) {
            speed_pid = 0;
            angle_pd  = 90;
            ROS_INFO_THROTTLE(2.0, "Estacionado correctamente.");
        }
    }

    void run() {
        task_assigner();
        task_solver();
    }

    // ── Motion helpers ────────────────────────────────────────────────────────
    void on_lane() {
        u_angle   = static_cast<int>(kp_angle * dist_now +
                                     kd_angle * (dist_now - angle_last));
        angle_pd  = std::max(45, std::min(135, 90 + u_angle));
        u_speed   = static_cast<int>(kp_speed * dist_now);
        speed_pid = std::max(-435, std::min(0, -435 + abs(u_speed)));
        angle_last = angle_pd;
    }

    void publish_policies() {
        angle_message.data = angle_pd;
        angle_pub.publish(angle_message);
        speed_message.data = speed_pid;
        speed_pub.publish(speed_message);
    }
};


int main(int argc, char **argv) {
    ros::init(argc, argv, "Master_parking");
    Master *control = new Master(Task(LANE_DRIVING));
    ros::spin();
    return 0;
}
