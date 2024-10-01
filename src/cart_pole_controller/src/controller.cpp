#include <chrono>
#include <functional>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/wrench.hpp>

using namespace std::chrono_literals;

class CartPoleController : public rclcpp::Node
{
public:
    CartPoleController()
    : Node("cart_pole_controller")
    {
        // Subscriber per leggere lo stato del pendolo
        pole_state_subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/cart/pole_state", 10, std::bind(&CartPoleController::pole_state_callback, this, std::placeholders::_1));
        
        // Publisher per pubblicare la forza sul carrello
        force_publisher_ = this->create_publisher<geometry_msgs::msg::Wrench>("/cart/force", 10);

        // Timer per il controllo PID
        timer_ = this->create_wall_timer(
            50ms, std::bind(&CartPoleController::control_loop, this));

        // Parametri PID (modificabili durante il tuning)
        kp_ = 150.0;
        ki_ = 0.1;
        kd_ = 20.0;

        prev_error_ = 0.0;
        integral_ = 0.0;
        pole_angle_ = 0.0;
    }

private:
    void pole_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        // Legge l'angolo del pendolo dal messaggio
        pole_angle_ = msg->position[0];
    }

    void control_loop()
    {
        // Calcola l'errore (distanza dall'angolo verticale, 0.0 radianti)
        double error = 0.0 - pole_angle_;
        
        // Proporzionale
        double p_term = kp_ * error;

        // Integrale
        integral_ += error * 0.05;  // 0.05 secondi
        double i_term = ki_ * integral_;

        // Derivativo
        double derivative = (error - prev_error_) / 0.05;
        double d_term = kd_ * derivative;

        // Output PID
        double control_output = p_term + i_term + d_term;

        // Imposta un limite alla forza per evitare accelerazioni eccessive
        control_output = std::max(std::min(control_output, 10.0), -10.0);

        // Aggiorna l'errore precedente
        prev_error_ = error;

        // Pubblica la forza sul carrello
        auto force_msg = geometry_msgs::msg::Wrench();
        force_msg.force.y = control_output;
        force_publisher_->publish(force_msg);
        
         // Log di debug per monitorare l'andamento
        RCLCPP_INFO(this->get_logger(), "Angle: %.3f, Control Output: %.3f", pole_angle_, control_output);
    }

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr pole_state_subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::Wrench>::SharedPtr force_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    double pole_angle_;
    double kp_, ki_, kd_;
    double prev_error_, integral_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CartPoleController>());
    rclcpp::shutdown();
    return 0;
}
