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
#include <sensor_msgs/Range.h>

// ── Estados ───────────────────────────────────────────────────────────────────
//  LANE_DRIVING     : avanza siguiendo la línea, buscando el cajón
//  ALIGN_TO_SPACE   : avanza ALIGN_TIME_MS extra para alinear eje trasero
//  PARKING_IN       : retrocede girando a la derecha hasta detectar banqueta
//  STOP             : estacionado
//
// Lógica de detección del cajón (flanco):
//   1. ultra_right < NEAR_THRESHOLD  → hay coche a la derecha
//   2. ultra_right > SPACE_THRESHOLD → el coche ya pasó = cajón vacío detectado
//
// Dimensiones cajón TMR 2026: 40 cm ancho × 60 cm fondo
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
static const float NEAR_THRESHOLD  = 0.20f; // "hay coche al lado derecho"
static const float SPACE_THRESHOLD = 0.35f; // "ya no hay coche = cajón"
static const float WALL_THRESHOLD  = 0.10f; // banqueta trasera
static const float LEFT_SAFE_MIN   = 0.08f; // seguridad anti-choque izquierdo

static const long  ALIGN_TIME_MS   = 600;

// ── Velocidades ───────────────────────────────────────────────────────────────
static const int BASE_SPEED        = -380;  // velocidad crucero
static const int MIN_SPEED         = -200;  // velocidad mínima al curvar fuerte


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
    ros::Subscriber ultrasonic_left_sub;

    // Publishers
    ros::Publisher angle_pub;
    ros::Publisher speed_pub;

    // Watchdog timer
    ros::Timer watchdog_timer_;
    ros::Time  last_cmd_time_;

    std_msgs::Int16 angle_message;
    std_msgs::Int16 speed_message;

    // Lane following
    int   dist_now;
    int   angle_last;
    float kp_angle;
    float kd_angle;
    float kp_speed;
    int   u_angle;
    int   angle_pd;
    int   speed_pid;
    int   u_speed;

    // Task management
    std::vector<Task> task_pile;

    // Ultrasonidos
    float ultra_right;
    float ultra_back;
    float ultra_left;

    // Detección por flanco
    bool saw_car_on_right;
    bool space_detected_once;

    // Timer ALIGN_TO_SPACE
    std::chrono::steady_clock::time_point align_start;

public:
    Master(Task task) {
        distance_center_sub  = nh_.subscribe("/distance_center_line", 1,
                                              &Master::dist_center_clbk, this);
        ultrasonic_right_sub = nh_.subscribe("/sensors/ultrasonic/right", 1,
                                              &Master::ultra_right_clbk, this);
        ultrasonic_back_sub  = nh_.subscribe("/sensors/ultrasonic/back",  1,
                                              &Master::ultra_back_clbk,  this);
        ultrasonic_left_sub  = nh_.subscribe("/sensors/ultrasonic/left",  1,
                                              &Master::ultra_left_clbk,  this);

        angle_pub = nh_.advertise<std_msgs::Int16>(
                        "/AutoModelMini/manual_control/steering", 1000);
        speed_pub = nh_.advertise<std_msgs::Int16>(
                        "/AutoModelMini/manual_control/speed",    1000);

        // Watchdog: si la cámara pierde frames por más de 300 ms,
        // mantiene una velocidad mínima segura para que el carro
        // no se quede sin comandos y se detenga inesperadamente
        watchdog_timer_ = nh_.createTimer(
            ros::Duration(0.1),
            &Master::watchdog_clbk, this
        );

        dist_now  = angle_last = 0;
        kp_angle  = 0.825f;
        kd_angle  = 0.0297f;
        kp_speed  = 1.2645f;
        u_angle   = angle_pd = speed_pid = u_speed = 0;

        ultra_right         = 4.0f;
        ultra_back          = 4.0f;
        ultra_left          = 4.0f;
        saw_car_on_right    = false;
        space_detected_once = false;

        last_cmd_time_ = ros::Time::now();

        this->add_task(task);
        ROS_INFO("Master_parking listo - esperando coche a la derecha...");
    }

    // ── Watchdog ──────────────────────────────────────────────────────────────
    // Se dispara cada 100 ms. Si no llegan datos de lane_detection por más de
    // 300 ms, publica velocidad mínima segura para que el carro no se detenga.
    void watchdog_clbk(const ros::TimerEvent&) {
        double elapsed = (ros::Time::now() - last_cmd_time_).toSec();
        if (elapsed > 0.3) {
            ROS_WARN_THROTTLE(1.0,
                "Sin datos de lane_detection (%.2f s) - velocidad minima", elapsed);
            speed_pid = MIN_SPEED;
            angle_pd  = 90;
            publish_policies();
        }
    }

    // ── Callbacks ──────────────────────────────────────────────────────────────
    void ultra_right_clbk(const sensor_msgs::Range::ConstPtr& msg) {
        ultra_right = msg->range;
    }
    void ultra_back_clbk(const sensor_msgs::Range::ConstPtr& msg) {
        ultra_back = msg->range;
    }
    void ultra_left_clbk(const sensor_msgs::Range::ConstPtr& msg) {
        ultra_left = msg->range;
    }

    void dist_center_clbk(const std_msgs::Int16& dis_now_center) {
        last_cmd_time_ = ros::Time::now(); // reinicia el watchdog
        dist_now = static_cast<int>(dis_now_center.data);
        this->run();
        this->publish_policies();
    }

    // ── Task management ────────────────────────────────────────────────────────
    void add_task(Task task)  { task_pile.push_back(task); }
    void remove_task() {
        if (task_pile.size() > 1) task_pile.pop_back();
    }
    Task get_current_task() { return task_pile.back(); }

    // ── task_assigner: detección por flanco ────────────────────────────────────
    //
    //  FASE 1 — saw_car_on_right = false:
    //    Espera que ultra_right < NEAR_THRESHOLD (coche detectado a la derecha).
    //
    //  FASE 2 — saw_car_on_right = true:
    //    Espera que ultra_right > SPACE_THRESHOLD (el coche pasó = cajón vacío).
    //
    //  Esto garantiza que el carrito haya avanzado junto al coche vecino
    //  ANTES de detectar el espacio vacío, evitando el arranque inmediato.
    // ──────────────────────────────────────────────────────────────────────────
    void task_assigner() {
        if (get_current_task().ID != LANE_DRIVING) return;
        if (space_detected_once) return;

        // FASE 1: esperar ver un coche a la derecha
        if (!saw_car_on_right) {
            if (ultra_right < NEAR_THRESHOLD) {
                saw_car_on_right = true;
                ROS_INFO("Coche derecho detectado (%.2f m) - esperando cajón...",
                         ultra_right);
            }
            return;
        }

        // FASE 2: el coche ya pasó → cajón vacío detectado
        if (ultra_right > SPACE_THRESHOLD) {
            space_detected_once = true;
            align_start = std::chrono::steady_clock::now();
            ROS_INFO("Cajon detectado (derecho=%.2f m) - alineando eje trasero...",
                     ultra_right);
            add_task(Task(ALIGN_TO_SPACE));
        }
    }

    // ── task_solver ────────────────────────────────────────────────────────────
    void task_solver() {
        Task current = get_current_task();
        ROS_INFO_STREAM_THROTTLE(0.5,
            "[Estado]: " << current.name
            << " | der=" << ultra_right << "m"
            << " | tra=" << ultra_back  << "m"
            << " | izq=" << ultra_left  << "m");

        // LANE DRIVING: sigue la línea mientras busca el cajón
        if (current.ID == LANE_DRIVING) {
            on_lane();
        }

        // ALIGN TO SPACE: avanza ALIGN_TIME_MS ms para alinear eje trasero
        else if (current.ID == ALIGN_TO_SPACE) {
            on_lane();
            long elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                               std::chrono::steady_clock::now() - align_start).count();
            if (elapsed >= ALIGN_TIME_MS) {
                ROS_INFO("Alineado - iniciando entrada al cajon");
                remove_task();
                add_task(Task(PARKING_IN));
            }
        }

        // PARKING IN: retrocede girando a la derecha
        else if (current.ID == PARKING_IN) {

            // Seguridad izquierda: si el morro se acerca demasiado al coche
            // vecino izquierdo durante la maniobra, pausamos la reversa
            if (ultra_left < LEFT_SAFE_MIN) {
                ROS_WARN_THROTTLE(0.3,
                    "Muy cerca del coche izquierdo (%.2f m) - pausando reversa",
                    ultra_left);
                speed_pid = 0;
                angle_pd  = 90;
                return;
            }

            angle_pd  = 45;   // giro derecha en reversa (ajustar en pista)
            speed_pid = 250;  // reversa

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

    // ── on_lane: velocidad con piso garantizado ────────────────────────────────
    //
    //  ANTES: speed_pid podía llegar a 0 si dist_now era grande,
    //         deteniendo el carro cada vez que el offset aumentaba.
    //
    //  AHORA: velocidad BASE_SPEED en recta, se reduce al curvar pero
    //         nunca baja de MIN_SPEED, así el carro siempre avanza.
    void on_lane() {
        u_angle   = static_cast<int>(kp_angle * dist_now +
                                     kd_angle * (dist_now - angle_last));
        angle_pd  = std::max(45, std::min(135, 90 + u_angle));

        int speed_reduction = static_cast<int>(kp_speed * abs(dist_now));
        speed_pid = std::max(MIN_SPEED,
                             std::min(BASE_SPEED, BASE_SPEED + speed_reduction));

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
