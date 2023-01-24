#!/usr/bin/env python3
import rclpy
import random # do losowania
import math # do uzycia pi
from rclpy.node import Node
from functools import partial # import potrzbny przy serwerze

from turtlesim.srv import Spawn # importowanie uslugi tworzenia nowych zolwi
from turtlesim.srv import Kill # importowanie uslugi zabijania zolwi
from interfejsy.msg import Turtle  # import interfejsu do przechowywania zmiennych na temat zolwia
from interfejsy.msg import TurtleArray # import interfejsu do przechowywania tablicy zolwi na planszy
from interfejsy.srv import  CatchTurtle # import uslugi sprawdzenia zlapania zolwia


class SpawnerNode(Node):
    def __init__(self):
        super().__init__("spawner")
        self.counter = 1 # numer zolwia
        self.objects_to_catch = [] # array dla zolwi
        self.objects_to_catch_pub = self.create_publisher(TurtleArray, "objects_to_catch", 10)
        # publishuje array zolwi na temacie objects_to_catch
        self.spawn_timer = self.create_timer(2, self.spawn) # tworzenie nowego zolwia co 2 sekundy
        self.service_catch = self.create_service(CatchTurtle, "catch", self.server_response) # tworzenie uslugi sprawdzenia cze zolw zlapany

    def server_response(self, request, response):
        self.kill_server(request.name) # uruchomienie serwera do zabijania zolwia
        response.hit = True # odpowiedz true serwera do lapania zolwi
        return response

    def pub_objects_to_catch(self):
        msg = TurtleArray() # msg jako interfejs tablicy zolwi
        msg.turtles = self.objects_to_catch  # przypisanie do wiadomosci naszej tablicy zolwi
        self.objects_to_catch_pub.publish(msg) #publikacja tej tablicy na temacie

    def spawn(self):
        self.counter += 1 # numer zwiekszany o 1
        name = "turtle" + str(self.counter) # zolw + numer
        x = random.uniform(0.0, 11.0) # losowanie wspolrzednej x
        y = random.uniform(0.0, 11.0) # losowanie wspolrzednej y
        theta = random.uniform(0.0, 2 * math.pi) # losowanie kata theta
        self.server_spawn(name, x, y, theta) # wyslanie danych na serwer w celu stworzenia zolwia

    def server_spawn(self, turtle_name, x, y, theta):
        client = self.create_client(Spawn, "spawn") # tworzenie klienta
        while not client.wait_for_service(1.0): # kiedy klient oczekuje na usluge to waiting
            self.get_logger().warn("Waiting for Server spawn...")

        # klient daje zmienne do tworzenia obiektu Spawn do serwera
        request = Spawn.Request() 
        request.x = x
        request.y = y
        request.theta = theta
        request.name = turtle_name

        future = client.call_async(request) # asynchroniczne wezwanie od klienta z jego prosba
        future.add_done_callback(
            partial(self.spawn_msg, turtle_name = turtle_name, x=x, y=y, 
            theta = theta)) # serwer odpowiada za pomoc spawn_msg

    def spawn_msg(self, future, turtle_name, x, y, theta):
        try:
            response = future.result() # odpowiedz serwera rowna wynikowi
            if response.name != "":
                self.get_logger().info("Turtle " + response.name + " showed up on screen") # log informujacy o dzialaniu, podaje nazwe zolwia o ile istnieje
                new = Turtle() # zmienna typu zolw
                # przypisanie danych do zmiennej new
                new.name = turtle_name
                new.x = x
                new.y = y
                new.theta = theta
                self.objects_to_catch.append(new) # dodanie zolwia do tablicy
                self.pub_objects_to_catch()
        except Exception as e:
            self.get_logger().error("Server failed %r" % (e,))

    def kill_server(self, turtle_name):
        client = self.create_client(Kill, "kill") # tworzenie klienta zabijania
        while not client.wait_for_service(1.0): # kiedy klient oczekuje na usluge to waiting
            self.get_logger().warn("Waiting for Server Spawner...")

        # klient daje zmienne do tworzenia obiektu Spawn do serwera
        request = Kill.Request() 
        request.name = turtle_name

        future = client.call_async(request) # asynchroniczne wezwanie od klienta z jego prosba
        future.add_done_callback(
            partial(self.kill_msg, turtle_name = turtle_name)) # serwer odpowiada za pomoc kill_msg

    def kill_msg(self, future, turtle_name):
        try:
            future.result() # odpowiedz serwera rowna wynikowi
            for (i, turtle) in enumerate(self.objects_to_catch):
                if turtle.name == turtle_name:
                    del self.objects_to_catch[i] # usuniecie zolwia o danym imieniu po zlapaniu
                    self.pub_objects_to_catch() # publikacja tablicy
                    break
        except Exception as e:
            self.get_logger().error("Server failed %r" % (e,))

def main(args=None):
    rclpy.init(args=args)
    node = SpawnerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
