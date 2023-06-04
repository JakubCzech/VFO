import math
import os
import sys
import numpy as np
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import rclpy
from python_vfo import atan2c, BSP20
from python_vfo.GSR_PF_nonP import GSR
from tf_transformations import euler_from_quaternion


class VFO(Node):
    def __init__(self):
        super().__init__("vfo")
        self.create_subscriber()

        self.declare_parameters(
            namespace="",
            parameters=[
                ("sigma", 1.0),
                ("vmax", 1.5),
                ("ksid", 1.0),
                ("Kp", 4.0),
                ("Ka", 8.0),
                ("Vpath", 0.5),
                ("path_type", os.environ.get("PATH_TYPE", "ellipse")),
            ],
        )
        self.phi = 0.0

        self.path_type = self.get_parameter("path_type").value
        self.sigma = self.get_parameter("sigma").value
        self.vmax = self.get_parameter("vmax").value
        self.ksid = self.get_parameter("ksid").value
        self.Kp = self.get_parameter("Kp").value
        self.Ka = self.get_parameter("Ka").value
        self.Vpath = self.get_parameter("Vpath").value
        self.get_logger().info("Path type: %s" % self.path_type)
        self._pub_twist = self.create_publisher(Twist, "/cmd_vel", 10)
        self.counter = 0
        self.error = 0.0
        self.create_fun()

    def create_subscriber(self):
        self._sub_odom = self.create_subscription(
            Odometry,
            "/odom",
            self.__odom_callback,
            10,
        )

    def __odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.theta = euler_from_quaternion(
            [
                msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z,
                msg.pose.pose.orientation.w,
            ]
        )[2]
        if self.counter >= 600:
            self.reset()

        self.calculate_velocity()
        self.counter += 1

    def reset(self):
        self._pub_twist.publish(Twist())
        self.get_logger().info(
            f"Error: {self.error/self.counter}, KP: {self.Kp}, KA: {self.Ka}, Vpath: {self.Vpath}"
        )
        with open("/root/workspace/src/log.txt", "a") as f:
            f.write(f"{self.error/self.counter}, {self.Kp}, {self.Ka}, {self.Vpath}")
            f.write("\n")
        self.counter = 0.0
        self.error = 0.0
        self._pub_twist.publish(Twist())
        # reset simulation
        sys.exit(1)

    def create_fun(self):
        self._gsr = GSR(self.sigma)
        self.F, self.Fx, self.Fxx, self.Fy, self.Fyy, self.Fxy = self._gsr.test()
        return
        if self.path_type == "ellipse":
            (
                self.F,
                self.Fx,
                self.Fxx,
                self.Fy,
                self.Fyy,
                self.Fxy,
            ) = self._gsr.calc_lambda_ellipse()
        elif self.path_type == "superellipse":
            (
                self.F,
                self.Fx,
                self.Fxx,
                self.Fy,
                self.Fyy,
                self.Fxy,
            ) = self._gsr.calc_lambda_superellipse()
        elif self.path_type == "tangen":
            (
                self.F,
                self.Fx,
                self.Fxx,
                self.Fy,
                self.Fyy,
                self.Fxy,
            ) = self._gsr.calc_lambda_tangens()
        else:
            raise ValueError("Wrong path type")

    def calculate_velocity(self):
        # Calculate functions values
        F = self.F(self.x, self.y)
        Fx = self.Fx(self.x, self.y)
        Fy = self.Fy(self.x, self.y)
        Fxx = self.Fxx(self.x, self.y)
        Fyy = self.Fyy(self.x, self.y)
        Fxy = self.Fxy(self.x, self.y)
        self.error += F
        # VFO algorithm
        NEGnablaF = np.array([-Fx, -Fy])
        NnablaF = np.linalg.norm(NEGnablaF, ord=2)
        WersorNEGnablaF = NEGnablaF / NnablaF
        # Define Rot and wt(p)
        Rot = np.array([[0, 1], [-1, 0]])  #  matrixrotation
        # Define WersorNEGnablaFORT
        NEGnablaFORT = Rot @ NEGnablaF
        WersorNEGnablaFORT = NEGnablaFORT / NnablaF
        # Define H, H1, and H2

        H = self.Vpath * WersorNEGnablaFORT + self.Kp * F * WersorNEGnablaF
        H1, H2 = H[0], H[1]
        # Define u2
        teta = self.theta
        self.get_logger().debug("teta: %s" % teta)
        u2 = H1 * math.cos(teta) + H2 * math.sin(teta)
        # Define tetaa, Oldtetaa, and EtetaaGlogow
        tetaa = math.atan2(self.ksid * H2, self.ksid * H1)  # orientacja pomocnicza
        tetaa = atan2c(tetaa, self.phi)
        self.get_logger().debug("tetaa: %s" % tetaa)
        self.phi = tetaa
        Etetaa = tetaa - teta
        xp = u2 * np.cos(teta)
        yp = u2 * np.sin(teta)
        dFdt = Fx * xp + Fy * yp
        dWerdt1 = (
            Fy
            * ((Fx * Fxy - Fy * Fxx) * xp + (Fx * Fyy - Fy * Fxy) * yp)
            / np.power(np.sqrt(Fx**2 + Fy**2), 3)
        )
        dWerdt2 = (
            Fx
            * ((Fy * Fxx - Fx * Fxy) * xp + (Fy * Fxy - Fx * Fyy) * yp)
            / np.power(np.sqrt(Fx**2 + Fy**2), 3)
        )
        dWersorNEGnablaFdt = np.array([dWerdt1, dWerdt2])
        Hp = self.Vpath * Rot @ dWersorNEGnablaFdt + self.Kp * (
            dFdt * WersorNEGnablaF + F * dWersorNEGnablaFdt
        )
        H1p, H2p = Hp[0], Hp[1]
        tetaap = (H2p * H1 - H2 * H1p) / (H1**2 + H2**2)
        u1 = self.Ka * Etetaa + tetaap
        # Scale velocity
        u1, u2 = BSP20(u1, u2, self.vmax)
        # Publish velocity
        _twist = Twist()
        _twist.linear.x = u1
        _twist.angular.z = u2
        self.get_logger().debug(f"u1: {u1}, u2: {u2}")
        self._pub_twist.publish(_twist)


def main(args=None):
    rclpy.init(args=args)
    vfo = VFO()
    try:
        rclpy.spin(vfo)
    except KeyboardInterrupt:
        pass
    finally:
        vfo.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
