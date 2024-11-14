from demo_python_node.person_node import PersonNode

class WriterNode(PersonNode):
    def __init__(self,name:str,age:int,book_value: str) ->None:
        print('WriteNode __init__')
        super().__init__(name,age)
        self.book = book_value


def main():
    node = WriterNode('小王',32,'食谱')
    node.eat('牢饭')