#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16MultiArray, String #int16 er meldingstypen for to heltall (hastighwetene), og string er meldingstype for bryterstatus

class BevegelsesNode(Node):
    def __init__(self):
        super().__init__('bevegelsesnode')
        self.sub = self.create_subscription(String, 'bryter/status', self.status_callback, 10) #lytter på topic /bryter/status, callbackfunksjonen kalles på hver gang en melding kommer inn, 10 er kø-størrelsen
        self.pub = self.create_publisher(Int16MultiArray, 'bevegelses/kommando', 10) #publiserer til /bevegelse/kammando med to heltall [venstre, høyre]. Motor-driveren abonnerer på det
        self.timer_pirouette = None #plass for å lagre timer-objekt, så vi kan hindre flere pirouetter
        self.get_logger().info("Bevegelsesnode startet og venter på bryter‐endringer.") #vanlig logging

    def status_callback(self, msg: String): #kjører når bryterstatus endrer seg
        if msg.data == 'ON':
            self.get_logger().info("Bryter ON oppdaget – starter pirouette om 1 sekund.")
            #venter 1 sek før pirouette
            if self.timer_pirouette:
                self.timer_pirouette.cancel() #kansellerer eventuellt eksisterende timer
            self.timer_pirouette = self.create_timer(1.0, self.pirouette) #lager engangs-timer (create_timer funksjon arves fra Node, om det er uklart)
        else:
            self.get_logger().info("Bryter OFF – stopper bevegelse.") #ellers status OFF
            #send stopp‐kommando
            cmd = Int16MultiArray()
            cmd.data = [0, 0] #av til motorene
            self.pub.publish(cmd)
    def pirouette(self): #kalles 1 sek etter ON
        if self.timer_pirouette:
            self.timer_pirouette.cancel() #kansellerer sin egen timer, slik at den bare kjører én gang
            self.timer_pirouette = None

        cmd = Int16MultiArray()
        cmd.data = [50, -50] #venstre hjul +50 og høyre hjul -50: 360 grader priouette
        self.pub.publish(cmd)
        self.get_logger().info("Pirouette pågår – hastighet på hjul [50, -50]") 

        #Etter 3 sek, send stopp
        self.create_timer(3.0, self.stop_motors, callback_group=None)

    def stop_motors(self): #funksjon for å stoppe motorene
        cmd = Int16MultiArray()
        cmd.data = [0, 0]
        self.pub.publish(cmd)
        self.get_logger().info("Pirouette ferdig – stopper hjulene.")

def main():
    rclpy.init() #starter ROS-klienten
    node = BevegelsesNode() # oppretter noden
    try:
        rclpy.spin(node) #Kaller callbakcs så lenge programmet kjører
	pass
    finally: #sikrer opprydding
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
