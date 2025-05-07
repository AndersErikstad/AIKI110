#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16MultiArray
import RPi.GPIO as GPIO

class MotorDriver(Node):
    def __init__(self):
        super().__init__('motor_driver')

        #pin definisjoner bcm
        self.LEFT_FWD_PIN=6
        self.LEFT_BWD_PIN=13
        self.RIGHT_FWD_PIN=19
        self.RIGHT_BWD_PIN=26

        #pwm frekvens i hz
        self.PWM_FREQ=1000

        #gjør klar gpio
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        for pin in (
            self.LEFT_FWD_PIN,
            self.LEFT_BWD_PIN,
            self.RIGHT_FWD_PIN,
            self.RIGHT_BWD_PIN
        ):
            GPIO.setup(pin, GPIO.OUT)

        #opprett fire pwm kanaler
        self.pwm_left_fwd=GPIO.PWM(self.LEFT_FWD_PIN, self.PWM_FREQ)
        self.pwm_left_bwd=GPIO.PWM(self.LEFT_BWD_PIN, self.PWM_FREQ)
        self.pwm_right_fwd=GPIO.PWM(self.RIGHT_FWD_PIN, self.PWM_FREQ)
        self.pwm_right_bwd=GPIO.PWM(self.RIGHT_BWD_PIN, self.PWM_FREQ)

        for pwm in (
            self.pwm_left_fwd,
            self.pwm_left_bwd,
            self.pwm_right_fwd,
            self.pwm_right_bwd
        ):
            pwm.start(0)  #start 0 duty hold low til vi endrer

        #abonner topic bevegelses kommando
        self.create_subscription(
            Int16MultiArray,
            'bevegelses/kommando',
            self.kjør_callback,
            10
        )
        self.get_logger().info('motordriver klar lytter på bevegelses/kommando')

    def kjør_callback(self, msg: Int16MultiArray):
        #sjekk at vi har to tal
        if len(msg.data)<2:
            self.get_logger().warning('forventer to elementer fikk %s', msg.data)
            return

        venstre,høyre=msg.data[:2]

        #kjør hjul hver for seg med pwm
        self._drive_wheel(self.pwm_left_fwd, self.pwm_left_bwd, venstre, side='venstre')
        self._drive_wheel(self.pwm_right_fwd,self.pwm_right_bwd,høyre,   side='høyre')
        self.get_logger().info(f'hastighet venstre={venstre} høyre={høyre}')

    def _drive_wheel(self,pwm_fwd,pwm_bwd,speed: int,*,side: str):
        #begrens speed til minus100 til 100
        speed=max(min(speed,100),-100)

        if speed>0:
            pwm_bwd.ChangeDutyCycle(0)
            pwm_fwd.ChangeDutyCycle(speed)
        elif speed<0:
            pwm_fwd.ChangeDutyCycle(0)
            pwm_bwd.ChangeDutyCycle(-speed)
        else:
            pwm_fwd.ChangeDutyCycle(0)
            pwm_bwd.ChangeDutyCycle(0)

    def destroy_node(self):
        #stopp pwm og rydd gpio
        self.get_logger().info('stopper pwm rydder gpio')
        for pwm in (
            self.pwm_left_fwd,
            self.pwm_left_bwd,
            self.pwm_right_fwd,
            self.pwm_right_bwd
        ):
            pwm.stop()
        GPIO.cleanup()
        return super().destroy_node()

def main():
    rclpy.init()
    node=MotorDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__=='__main__':
    main()
