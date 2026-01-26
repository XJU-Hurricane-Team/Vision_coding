import rclpy
from rclpy.node import Node
class PersonNode(Node):
    def __init__(self,node_name:str,name:str,age:int)->None:
        super().__init__(node_name)
        self.age = age
        self.name = name
    
    def eat(self,food_name:str):
        self.get_logger().info(f' 我叫 {self.name}，今年 {self.age} 岁，我现在正在吃 {food_name}')

def main():
    rclpy.init()
    node = PersonNode('person_node','小明',20)
    node.eat('苹果')
    rclpy.spin(node)
    rclpy.shutdown()