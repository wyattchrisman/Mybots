from solution import SOLUTION
import constants as c
import copy
import os

class PARALLEL_HILL_CLIMBER:

    def __init__(self):

        os.system("rm brain*.nndf")
        os.system("rm fitness*.txt")

        self.parents = {}
        self.nextAvailableID = 0

        for i in range(c.populationSize):
            self.parents[i] = SOLUTION(self.nextAvailableID)
            self.nextAvailableID += 1


    def Evolve(self):
        '''
        self.parent.Evaluate("GUI")

        '''
        self.Evaluate(self.parents)

        for currentGeneration in range(c.numberOfGenerations):
            self.Evolve_For_One_Generation()

    def Evaluate(self, solutions):
        for parent in solutions.values():
            parent.Start_Simulation("DIRECT")

        for parent in solutions.values():
            parent.Wait_For_Simulation_To_End()
            # print(f"\nFITNESS of {parent.myID} = {parent.fitness}")

    def Evolve_For_One_Generation(self):
        self.Spawn()
        self.Mutate()
        self.Evaluate(self.children)
        self.Print()
        self.Select()

    def Show_Best(self):
        '''
        self.parent.Evaluate("GUI")
        '''
        best = self.parents[0]
        for parent in self.parents.values():
            if parent.fitness < best.fitness:
                best = parent

        best.Start_Simulation("GUI")

        

    def Print(self):
        for key in self.parents.keys():
            print(f"\nParent {key}: {self.parents[key].fitness} Child: {self.children[key].fitness}\n")

    def Spawn(self):
        self.children = {}

        for key in self.parents.keys():
            self.children[key] = copy.deepcopy(self.parents[key])
            self.children[key].Set_ID(self.nextAvailableID)
            self.nextAvailableID += 1
    
    def Mutate(self):
        for child in self.children.values():
            child.Mutate()

    def Select(self):
        for p_idx, parent in self.parents.items():
            if parent.fitness > self.children[p_idx].fitness:
                self.parents[p_idx] = self.children[p_idx]
