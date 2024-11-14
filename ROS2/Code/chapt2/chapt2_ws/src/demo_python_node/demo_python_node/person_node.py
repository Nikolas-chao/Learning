import rclpy
from rclpy.node import Node

class PersonNode(Node):
    def __init__(self,node_name:str,name_value: str,age_value: int) ->None:
        
        super().__init__(node_name)
        print('PersonNode __init__')
        self.name = name_value
        self.age = age_value

    def eat(self,food_name: str):
        """
        方法：吃东西
        :food_name 食物名字
        """
        self.get_logger().info(f"{self.name},{self.age}岁,爱吃{food_name}")
        # print(f"{self.name},{self.age}岁,爱吃{food_name}")


def main():
    rclpy.init()

    node = PersonNode("xiaowang","小王",36)
    node.eat("紫菜蛋花汤")

    rclpy.spin(node)
    rclpy.shutdown()
    