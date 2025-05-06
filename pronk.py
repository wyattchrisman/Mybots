from solution import SOLUTION
import constants as c
import copy
import csv
import os

class PRONK:

    def __init__(self):

        os.system("rm brain*.nndf")
        os.system("rm fitness*.txt")
        os.system("rm oscillations*.txt")

        self.parents = {}
        self.nextAvailableID = 0

        for i in range(c.populationSize):
            self.nextAvailableID += 1
            self.parents[i] = SOLUTION(self.nextAvailableID)


    def Evolve(self):
        '''
        self.parent.Evaluate("GUI")

        '''
        self.Evaluate(self.parents)

        for currentGeneration in range(c.numberOfGenerations):
            print(f"Generation: {currentGeneration+1}")
            self.Evolve_For_One_Generation()

    def Evaluate(self, solutions):
        for parent in solutions.values():
            parent.Start_Simulation("DIRECT")
            parent.Wait_For_Simulation_To_End()

        '''
        for parent in solutions.values():
            sleep_count = parent.Wait_For_Simulation_To_End()
            if sleep_count > 300:
                self.nextAvailableID += 1
        ''' 
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
            if c.maximize:
                if parent.fitness >= best.fitness:
                    best = parent
            else:
                if parent.fitness <= best.fitness:
                    best = parent
 
        best.Start_Simulation("GUI")

        

    def Print(self):
        for key in self.parents.keys():
            print(f"\nParent {key}: {self.parents[key].fitness} Child: {self.children[key].fitness}\n")

    def Spawn(self):
        self.children = {}

        for key in self.parents.keys():
            self.children[key] = copy.deepcopy(self.parents[key])
            self.nextAvailableID += 1
            self.children[key].Set_ID(self.nextAvailableID)
            
    
    def Mutate(self):
        for child in self.children.values():
            child.Mutate()

    def Select(self):
        for p_idx, parent in self.parents.items():
            new = False
            if c.maximize: 
                if parent.fitness <= self.children[p_idx].fitness:
                    self.parents[p_idx] = self.children[p_idx]
                    new = True
            else:
                if parent.fitness >= self.children[p_idx].fitness:
                    self.parents[p_idx] = self.children[p_idx]
                    new = True

            current_bot = 'B'
            file_path = f'robot{current_bot}_oscillations.csv'
            with open(file_path, mode='a', newline='') as file:
                writer = csv.writer(file)
                if new:
                    writer.writerow([self.parents[p_idx].oscillation])
                else:
                    writer.writerow([self.children[p_idx].oscillation])

            file_path = f'robot{current_bot}_fitness.csv'
            with open(file_path, mode='a', newline='') as file:
                writer = csv.writer(file)
                writer.writerow([self.parents[p_idx].fitness])
