from enum import Enum


class intersectionType(Enum):
    Left_and_Foward = 1
    Right_and_Foward = 2
    T = 3
    Left = 4
    Right = 5
    Three_Way = 6


def setDict(dict):
    dict.clear
    dict[intersectionType.Left_and_Foward.name] = 0
    dict[intersectionType.Right_and_Foward.name] = 0
    dict[intersectionType.T.name] = 0
    dict[intersectionType.Left.name] = 0
    dict[intersectionType.Right.name] = 0
    dict[intersectionType.Three_Way.name] = 0
    
