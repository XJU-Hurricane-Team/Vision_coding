from demo_python_pkg.person_node import PersonNode
class WriterNode(PersonNode):
    def __init__(self,node_name:str,name:str, age:int, book:str)->None:
        super().__init__(node_name,name, age)
        print('WriterNode 的 __init__ 方法被调用了')
        self.book = book

def main():
    node = WriterNode('writer_node','张三', 25, '论快速入狱')
    node.eat('鱼香肉丝')