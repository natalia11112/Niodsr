#!/usr/bin/env python3
import math #modul matematyczny do przeliczania pierwiastka
import rclpy
from rclpy.node import Node
from functools import partial # import potrzbny przy serwerze

from turtlesim.msg import Pose #import interfejsu pozycji zolwia
from geometry_msgs.msg import Twist #import dla cmd_vel do nadawania predkosci
from interfejsy.msg import Turtle  # import interfejsu do przechowywania zmiennych na temat zolwia
from interfejsy.msg import TurtleArray # import interfejsu do przechowywania tablicy zolwi na planszy
from interfejsy.srv import  CatchTurtle # import uslugi sprawdzenia zlapania zolwia


class ControllerNode(Node): 
    def __init__(self):
        super().__init__("controller")
        self.catch_object = None # objekt do zlapania
        self.catch_object_first = True
        self.position = None # zmienna dla pozycji
        self.velocity_pub = self.create_publisher(Twist,
            "turtle1/cmd_vel", 10) # publisher predkosci
        self.position_sub = self.create_subscription(Pose, 
            "turtle1/pose", self.sub_position_msg, 10) # subscriber pozycji
        # subskrybent array zywych zolwi
        self.objects_to_catch_sub = self.create_subscription(TurtleArray, "objects_to_catch", self.objects_to_catch_msg, 10) 
        self.timer = self.create_timer(0.01, self.loop) # timer glownej petli
        
        
    def loop(self):
        if self.position == None or self.catch_object == None: # jezeli nie ma zadnej pozycji z subscribera lub nie ma zolwia do zlapania zwraca return
            return 
        dist_x = self.catch_object.x - self.position.x # odleglosc na osi x
        dist_y = self.catch_object.y - self.position.y # odleglosc na osi y
        dist_x_sqr = dist_x * dist_x
        dist_y_sqr = dist_y * dist_y
        distance = math.sqrt(dist_x_sqr + dist_y_sqr) # przeliczanie odleglosci z pitagorasa

        msg = Twist()

        if distance > 0.5:
            msg.linear.x = 1.5 * distance # pozycja, mozna dostroic mnozac razy liczbe, regulator
            target_theta = math.atan2(dist_y, dist_x) # kat orientacji wyliczony tg(alfa) = dist_y/dist_x, i atan2 aby otrzymac alfa

            difference = target_theta - self.position.theta # roznica katow cel-aktualny
            if difference > math.pi:
                difference -= 2*math.pi # jezeli wiecej niz pi odejmujemy 2pi, zeby nie bylo wiecej niz 180 stopni
            elif difference < -math.pi:
                difference += 2*math.pi

            msg.angular.z = 2.5 * difference # orientacja, mozna dostroic mnozac razy liczbe, regulator
        else: # chcemy zeby zolw stanal gdy dotrze do celu na mniej niz 0.5 
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self.turtle_server(self.catch_object.name) # lapanie zolwia po dotarciu do celu
            self.catch_object = None

        self.velocity_pub.publish(msg) # publikowanie predkosci zolwia

    def sub_position_msg(self, msg): # funkcja przypisujaca do zmiennej wiadomosc z subscribera
        self.position = msg

    def objects_to_catch_msg(self, msg):
        if len(msg.turtles) > 0:
            if self.catch_object_first:
                object_first = None
                object_first_distance = None
                # wyliczenie najblizszego zolwia i podanie go do zlapania
                for turtle in msg.turtles:
                    dist_x = turtle.x - self.position.x
                    dist_y = turtle.y - self.position.y
                    distance_x_sqr = dist_x * dist_x
                    distance_y_sqr = dist_y * dist_y
                    distance = math.sqrt(distance_x_sqr + distance_y_sqr)
                    if object_first == None or distance < object_first_distance:
                        object_first = turtle
                        object_first_distance = distance
                self.catch_object = object_first

            else:
                self.catch_object = msg.turtles[0] # pierwszy z tablicy
        

    
    def turtle_server(self, turtle_name):
        client = self.create_client(CatchTurtle, "catch") # tworzenie klienta zabijania
        while not client.wait_for_service(1.0): # kiedy klient oczekuje na usluge to waiting
            self.get_logger().warn("Waiting for Server Turtle Catcher...")

        # klient daje zmienne do tworzenia obiektu Spawn do serwera
        request = CatchTurtle.Request() 
        request.name = turtle_name

        future = client.call_async(request) # asynchroniczne wezwanie od klienta z jego prosba
        future.add_done_callback(partial(self.catch_msg, turtle_name = turtle_name)) # serwer odpowiada za pomoca catch_msg

    def catch_msg(self, future, turtle_name):
        try:
            response = future.result() # odpowiedz serwera rowna wynikowi
            if not response.hit: # jezeli nie zakonczone sukcesem
                self.get_logger().error("Turtle  " + str(turtle_name) + " not caught")
        except Exception as e:
            self.get_logger().error("Server failed %r" % (e,))

def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode() 
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
