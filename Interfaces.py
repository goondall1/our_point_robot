import abc
from abc import ABC, abstractmethod

class State(ABC):
    @abstractmethod
    def getCoordinates(self):
        raise NotImplementedError



class Obstacle(ABC):
    
    @abstractmethod
    def isInObs(self, state: State):
        raise NotImplementedError




