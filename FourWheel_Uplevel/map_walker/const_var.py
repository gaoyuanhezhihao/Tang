class _CONST:
    def __init__(self,name):
        self.name = name
    def __setattr__(self,name,value):
        if self.__dict__.has_key(name):
            raise TypeError, "(%s) is const"%(name)
        else:
            self.__dict__[name]=value


const = _CONST("global")
if __name__ == '__main__':
    import sys
    try:
        my_int = typecheck(1)
        my_int.x = 3
        my_int.y = 4
        my_int.z = 5
        my_int.h = 'hey'
    except:
        print sys.exc_type, sys.exc_value

    try:
        my_string = typecheck('')
        my_int.s = 'hello'
        my_int.t = 'byebye'
        my_int.u = 3
    except:
        print sys.exc_type, sys.exc_value

