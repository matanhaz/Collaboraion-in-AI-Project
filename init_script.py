import re
import time
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import os

CARRY_AGENT = 4
FREE_AGENT = 3
BRICK = 2
GROUND = 1

X = 5
Y = 5
Z = 4

class TermitesProject:

    def __init__(self, num_agents, shape, shape_size):
        # Matrix => Goal State

        self.cube = np.zeros((X, Y, Z), dtype='int')
        self.fig = plt.figure()
        self.path = 'C:\\Users\\matan\\Desktop\\toar\\semester h\\shituf peula\\planner\\ma-strips\\benchmarks\\logistics00-factored\\probLOGISTICS-4-0'
        self.plan_path = os.path.join('C:\\Users\\matan\\Desktop\\toar\\semester h\\shituf peula\\planner\\ma-strips\\bin\\debug','Plan.txtPlan.txt')
        for file in os.listdir(self.path):
            os.remove(os.path.join(self.path,file))

        self.cube[0][0][1] = BRICK
        self.cube[0][1][1] = BRICK
        self.cube[0][2][1] = BRICK
        self.cube[0][3][1] = BRICK
        self.cube[0][4][1] = BRICK


        self.cube[1][0][1] = BRICK
        self.cube[1][1][1] = BRICK
        self.cube[1][2][1] = BRICK
        self.cube[1][3][1] = BRICK
        self.cube[1][4][1] = BRICK


        self.cube[2][0][1] = BRICK
        self.cube[2][4][1] = BRICK
        self.NUM_AGENTS = num_agents
        self.SHAPE = shape
        self.SHAPE_SIZE = shape_size

        self.cube[4][4][1] = FREE_AGENT
        self.cube[4][0][1] = FREE_AGENT
        if self.NUM_AGENTS > 2:
            self.cube[4][1][1] = FREE_AGENT
        if self.NUM_AGENTS > 3:
            self.cube[4][3][1] = FREE_AGENT



    def generate(self):
        positions = []

        for x in range(X):
            for y in range(Y):
                self.cube[x][y][0] = GROUND

        for x in range(X):
            for y in range(Y):
                for z in range(Z):
                    positions.append("pos" + str(x) + str(y) + str(z))

        adjacent_horizontal = []
        for pos in positions:
            for other_pos in positions:
                if pos != other_pos and abs((int(pos[3])) - (int(other_pos[3]))) + abs((int(pos[4])) - (int(other_pos[4]))) == 1 and pos[5] == other_pos[
                    5]:
                    adjacent_horizontal.append(f"(adjacent_horizontal {pos} {other_pos})")

        adjacent_vertical_up = []
        for pos in positions:
            for other_pos in positions:
                if pos != other_pos and int(pos[3]) == int(other_pos[3]) and int(pos[4]) == int(other_pos[4]) and \
                        int(pos[5]) == int(other_pos[5]) - 1:
                    adjacent_vertical_up.append(f"(adjacent_vertical_up {pos} {other_pos})")

        adjacent_vertical_down = []
        for pos in positions:
            for other_pos in positions:
                if pos != other_pos and int(pos[3]) == int(other_pos[3]) and int(pos[4]) == int(other_pos[4]) and \
                        int(pos[5]) == int(other_pos[5]) + 1:
                    adjacent_vertical_up.append(f"(adjacent_vertical_down {pos} {other_pos})")

        At_brick = []
        agent_counter = 0

        freecells = []
        for x in range(X):
            for y in range(Y):
                for z in range(Z):
                    if self.cube[x][y][z] == 0:
                        freecells.append(f"(freecell pos{x}{y}{z})")
                    elif self.cube[x][y][z] == BRICK or self.cube[x][y][z] == GROUND:
                        At_brick.append(f"(atbrick pos{x}{y}{z})")
                    elif self.cube[x][y][z] == FREE_AGENT:
                        agent_counter += 1
                        At_brick.append(f"(atagent a{agent_counter} pos{x}{y}{z})")
                        At_brick.append(f"(freeagent a{agent_counter})")


        text = ''
        text += """(define (problem logistics-4-0)
        (:domain logistics)
        (:objects"""

        text += '\n'

        for pos in positions:
            text += str(pos) + " - location"
            text += '\n'
        for i in range(1, agent_counter+1):
            text += "a" + str(i) + " - agent"
            text += '\n'

        text += (""")
        (:init""")
        text += '\n'
        for at in At_brick:
            text += str(at)
            text += '\n'
        for a2t in freecells:
            text += str(a2t)
            text += '\n'
        for adj1 in adjacent_horizontal:
            text += str(adj1)
            text += '\n'
        for adj2 in adjacent_vertical_up:
            text += str(adj2)
            text += '\n'
        for adj3 in adjacent_vertical_down:
            text += str(adj3)
            text += '\n'




        text += (""")
        (:goal
        (and""")
        text += '\n'
        # goal state



        text += self.generate_goal()
        for i in range(1, agent_counter+1):
            text +=f"(freeagent a{str(i)})"
            text += '\n'


        text +=(""")
        )
        )""")

        for i in range(1, agent_counter+1):
            with open(os.path.join(self.path,f'problem-a{str(i)}.pddl'), 'w') as out:
                out.write(text)

        print(text)


    def vis(self):
        x, y, z = np.indices((X, Y, Z))
        ground = (z == 0)
        logic_exp_brick = False
        logic_exp_free_agents = False
        logic_exp_carry_agents = False

        for x_val in range(X):
            for y_val in range(Y):
                for z_val in range(Z):
                    if self.cube[x_val][y_val][z_val] == BRICK:
                        logic_exp_brick = logic_exp_brick | ((x == x_val) & (y == y_val) & (z == z_val))
                    elif self.cube[x_val][y_val][z_val] == FREE_AGENT:
                        logic_exp_free_agents = logic_exp_free_agents | ((x == x_val) & (y == y_val) & (z == z_val))
                    elif self.cube[x_val][y_val][z_val] == CARRY_AGENT:
                        logic_exp_carry_agents = logic_exp_carry_agents | ((x == x_val) & (y == y_val) & (z == z_val))

        brick = logic_exp_brick
        free_agents = logic_exp_free_agents
        carry_agents = logic_exp_carry_agents

        voxels = brick | ground | free_agents | carry_agents

        colors = np.empty(voxels.shape, dtype=object)
        colors[free_agents] = 'blue'
        colors[carry_agents] = 'orange'
        colors[brick] = 'red'
        colors[ground] = 'green'

        # and plot everything
        plt.ion()

        ax = plt.axes(projection='3d')
        ax.voxels(voxels, facecolors=colors, edgecolor='k')
        ax.axis('on')
        # manager = plt.get_current_fig_manager()
        # manager.window.showMaximized()
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()
        #time.sleep(4)
        plt.pause(0.1)
        # plt.close()
        plt.show()

    def show_results(self):
        time.sleep(3)
        with open(self.plan_path) as file:
            lines = file.readlines()
            for line in lines:
                tokens = re.split(' |\(|\)|\n', line)
                action = tokens[3]
                if action == 'agent_move':
                    src = [int(v) for v in list(tokens[5][3:])]
                    dst = [int(x) for x in list(tokens[7][3:])]

                    tmp = self.cube[src[0]][src[1]][src[2]]
                    self.cube[dst[0]][dst[1]][dst[2]] = tmp
                    self.cube[src[0]][src[1]][src[2]] = 0
                elif action == 'agent_climb':
                    src = [int(v) for v in list(tokens[5][3:])]
                    dst = [int(x) for x in list(tokens[7][3:])]

                    tmp = self.cube[src[0]][src[1]][src[2]]
                    self.cube[dst[0]][dst[1]][dst[2]] = tmp
                    self.cube[src[0]][src[1]][src[2]] = 0
                elif action == 'agent_slide':
                    src = [int(v) for v in list(tokens[5][3:])]
                    dst = [int(x) for x in list(tokens[8][3:])]

                    tmp = self.cube[src[0]][src[1]][src[2]]
                    self.cube[dst[0]][dst[1]][dst[2]] = tmp
                    self.cube[src[0]][src[1]][src[2]] = 0
                elif action == 'pickup-brick':
                    src = [int(v) for v in list(tokens[5][3:])]
                    dst = [int(x) for x in list(tokens[7][3:])]

                    self.cube[dst[0]][dst[1]][dst[2]] = 0
                    self.cube[src[0]][src[1]][src[2]] = CARRY_AGENT
                elif action == 'put-brick':
                    src = [int(v) for v in list(tokens[5][3:])]
                    dst = [int(x) for x in list(tokens[7][3:])]

                    self.cube[dst[0]][dst[1]][dst[2]] = BRICK
                    self.cube[src[0]][src[1]][src[2]] = FREE_AGENT
                self.vis()

        time.sleep(4)

    def generate_goal(self):
        text = ''
        if self.SHAPE == 'box':
            if self.SHAPE_SIZE in ('2x1x1','4x1x1','2x2x1','2x2x2'):
                text += ("          (atbrick pos211)\n")
                text += ("          (atbrick pos221)\n")
            if self.SHAPE_SIZE in ('4x1x1'):
                text += ("          (atbrick pos201)\n")
                text += ("          (atbrick pos231)\n")
            if self.SHAPE_SIZE in ('2x2x1','2x2x2'):
                text += ("          (atbrick pos311)\n")
                text += ("          (atbrick pos321)\n")
            if self.SHAPE_SIZE in ('2x2x2'):
                text += ("          (atbrick pos212)\n")
                text += ("          (atbrick pos222)\n")
                text += ("          (atbrick pos312)\n")
                text += ("          (atbrick pos322)\n")

        # elif self.SHAPE == 'cube':
        #     if self.SHAPE_SIZE in ('1','4' ,'8'):
        #         text += ("(atbrick pos211)\n")
        #         text += ("(atbrick pos221)\n")
        #     if self.SHAPE_SIZE in ('4'):
        #         text += ("(atbrick pos201)\n")
        #         text += ("(atbrick pos231)\n")
        #     if self.SHAPE_SIZE in ('2X2X1','2X2X2'):
        #         text += ("(atbrick pos311)\n")
        #         text += ("(atbrick pos321)\n")
        #     if self.SHAPE_SIZE in ('2X2X2'):
        #         text += ("(atbrick pos212)\n")
        #         text += ("(atbrick pos222)\n")
        #         text += ("(atbrick pos312)\n")
        #         text += ("(atbrick pos322)\n")
        elif self.SHAPE == 'staircase':
            if self.SHAPE_SIZE in ('1','4','9'):
                text += ("          (atbrick pos221)\n")
            if self.SHAPE_SIZE in ('4' ,'9'):
                text += ("          (atbrick pos211)\n")
                text += ("          (atbrick pos231)\n")
                text += ("          (atbrick pos222)\n")
            if self.SHAPE_SIZE in ('9'):
                text += ("          (atbrick pos201)\n")
                text += ("          (atbrick pos241)\n")
                text += ("          (atbrick pos212)\n")
                text += ("          (atbrick pos232)\n")
                text += ("          (atbrick pos223)\n")
        else:
            raise Exception

        return text

    def generate_domain(self):
        text = ''
        text += """(define (domain logistics)
	(:requirements :factored-privacy :typing)
	(:types
		location agent - object\n"""

        for i in range(1, self.NUM_AGENTS+1):
            text += f'      a{i} - agent\n'

        text += """     pos000 - location
		pos001 - location
		pos002 - location
		pos003 - location
		pos010 - location
		pos011 - location
		pos012 - location
		pos013 - location
		pos020 - location
		pos021 - location
		pos022 - location
		pos023 - location
		pos030 - location
		pos031 - location
		pos032 - location
		pos033 - location
		pos040 - location
		pos041 - location
		pos042 - location
		pos043 - location
		pos100 - location
		pos101 - location
		pos102 - location
		pos103 - location
		pos110 - location
		pos111 - location
		pos112 - location
		pos113 - location
		pos120 - location
		pos121 - location
		pos122 - location
		pos123 - location
		pos130 - location
		pos131 - location
		pos132 - location
		pos133 - location
		pos140 - location
		pos141 - location
		pos142 - location
		pos143 - location
		pos200 - location
		pos201 - location
		pos202 - location
		pos203 - location
		pos210 - location
		pos211 - location
		pos212 - location
		pos213 - location
		pos220 - location
		pos221 - location
		pos222 - location
		pos223 - location
		pos230 - location
		pos231 - location
		pos232 - location
		pos233 - location
		pos240 - location
		pos241 - location
		pos242 - location
		pos243 - location
		pos300 - location
		pos301 - location
		pos302 - location
		pos303 - location
		pos310 - location
		pos311 - location
		pos312 - location
		pos313 - location
		pos320 - location
		pos321 - location
		pos322 - location
		pos323 - location
		pos330 - location
		pos331 - location
		pos332 - location
		pos333 - location
		pos340 - location
		pos341 - location
		pos342 - location
		pos343 - location
		pos400 - location
		pos401 - location
		pos402 - location
		pos403 - location
		pos410 - location
		pos411 - location
		pos412 - location
		pos413 - location
		pos420 - location
		pos421 - location
		pos422 - location
		pos423 - location
		pos430 - location
		pos431 - location
		pos432 - location
		pos433 - location
		pos440 - location
		pos441 - location
		pos442 - location
		pos443 - location
	)

	(:predicates
		(atagent ?a - agent ?src - location)
		(atbrick ?dst - location)
		(freecell ?src - location)
		(freeagent ?a - agent)
		(adjacent_horizontal ?src - location ?dst - location)
		(adjacent_vertical_up ?loc - location ?loc - location)
	)

	(:action agent_move
		:parameters (?a - agent ?src - location ?belowdst - location ?dst - location)
		:precondition (and
			(atagent ?a ?src)
			(freecell ?dst)
			(adjacent_horizontal ?src ?dst)
			(adjacent_vertical_up ?belowdst ?dst)
			(atbrick ?belowdst)
		)
		:effect (and
			(atagent ?a ?dst)
			(not (atagent ?a ?src))
			(not (freecell ?dst))
			(freecell ?src)
		)
	)

	(:action agent_climb
		:parameters (?a - agent ?src - location ?base - location ?dst - location)
		:precondition (and
			(atagent ?a ?src)
			(freecell ?dst)
			(atbrick ?base)
			(adjacent_horizontal ?src ?base)
			(adjacent_vertical_up ?base ?dst)
		)
		:effect (and
			(atagent ?a ?dst)
			(not (atagent ?a ?src))
			(not (freecell ?dst))
			(freecell ?src)
		)
	)

	(:action agent_slide
		:parameters (?a - agent ?src - location ?standingbase - location ?base - location ?dst - location)
		:precondition (and
			(atagent ?a ?src)
			(freecell ?dst)
			(atbrick ?base)
			(atbrick ?standingbase)
			(adjacent_horizontal ?standingbase ?dst)
			(adjacent_vertical_up ?base ?dst)
			(adjacent_vertical_up ?standingbase ?src)
		)
		:effect (and
			(atagent ?a ?dst)
			(not (atagent ?a ?src))
			(not (freecell ?dst))
			(freecell ?src)
		)
	)

	(:action pickup-brick
		:parameters (?a - agent ?src - location ?above - location ?dst - location)
		:precondition (and
			(atbrick ?dst)
			(adjacent_horizontal ?src ?dst)
			(adjacent_vertical_up ?dst ?above)
			(freecell ?above)
			(atagent ?a ?src)
			(freeagent ?a)
		)
		:effect (and
			(not (freeagent ?a))
			(not (atbrick ?dst))
			(freecell ?dst)
		)
	)

	(:action put-brick
		:parameters (?a - agent ?src - location ?below - location ?dst - location)
		:precondition (and
			(adjacent_horizontal ?src ?dst)
			(atagent ?a ?src)
			(adjacent_vertical_up ?below ?dst)
			(atbrick ?below)
			(not (freeagent ?a))
			(freecell ?dst)
		)
		:effect (and
			(freeagent ?a)
			(atbrick ?dst)
			(not (freecell ?dst))
		)
	)

)"""

        for i in range(1, self.NUM_AGENTS+1):
            with open(os.path.join(self.path, f'domain-a{str(i)}.pddl'), 'w') as out:
                out.write(text)


    def visualize_shapes(self):
        self.cube = np.zeros((X, Y, Z), dtype='int')
        for x in range(X):
            for y in range(Y):
                self.cube[x][y][0] = GROUND

        if self.SHAPE == 'box':
            if self.SHAPE_SIZE in ('2x1x1','4x1x1','2x2x1','2x2x2'):
                self.cube[2][1][1] = BRICK
                self.cube[2][2][1] = BRICK
            if self.SHAPE_SIZE in ('4x1x1'):
                self.cube[2][0][1] = BRICK
                self.cube[2][3][1] = BRICK
            if self.SHAPE_SIZE in ('2x2x1','2x2x2'):
                self.cube[3][1][1] = BRICK
                self.cube[3][2][1] = BRICK
            if self.SHAPE_SIZE in ('2x2x2'):
                self.cube[2][1][2] = BRICK
                self.cube[2][2][2] = BRICK
                self.cube[3][1][2] = BRICK
                self.cube[3][2][2] = BRICK


        elif self.SHAPE == 'staircase':
            if self.SHAPE_SIZE in ('1','4','9'):
                self.cube[2][2][1] = BRICK
            if self.SHAPE_SIZE in ('4' ,'9'):
                self.cube[2][1][1] = BRICK
                self.cube[2][3][1] = BRICK
                self.cube[2][2][2] = BRICK

            if self.SHAPE_SIZE in ('9'):
                self.cube[2][0][1] = BRICK
                self.cube[2][4][1] = BRICK
                self.cube[2][1][2] = BRICK
                self.cube[2][3][2] = BRICK
                self.cube[2][2][3] = BRICK

        self.vis()
        time.sleep(5)



if __name__ == '__main__':
    project = TermitesProject(4, 'staircase', '9')
    # project.generate()
    # project.generate_domain()
    #project.show_results()
    project.visualize_shapes()

