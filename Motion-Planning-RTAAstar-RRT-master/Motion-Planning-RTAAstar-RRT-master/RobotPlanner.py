import numpy as np
import heapq


class RobotPlanner:
    __slots__ = ['boundary', 'blocks', 'counter','open_list','close_list','myGraph','expand_num','epsilon','visited','g']

    def __init__(self, boundary, blocks):
        self.boundary = boundary
        self.blocks = blocks
        self.counter = 0 
        self.open_list =[]
        self.close_list = []
        self.myGraph = {}
        self.expand_num = 20
        self.epsilon = 3
        self.visited = []
        self.g = 0 
        
    def computeHeuristic(self,start,goal):
        dis = sum((start-goal)**2)
        return dis
    
    def computeCost(self,dR):
        dis = sum(dR**2)
        return dis
    
    def check_inside(self, point, idx, b_idx):
        return point[idx] >= self.blocks[b_idx,idx] and point[idx] <= self.blocks[b_idx,3+idx] 
    
    def segmentCollision(self, start, end):
        valid = False
        for b in range(self.blocks.shape[0]):
            if (end[0] - self.blocks[b,0])*(start[0] - self.blocks[b,0]) <= 0 and \
                self.check_inside(end, 1, b) and self.check_inside(start, 1, b) and \
                self.check_inside(end, 2, b) and self.check_inside(start, 2, b):
                valid = True
            if (end[1] - self.blocks[b,1])*(start[1] - self.blocks[b,1]) <= 0 and \
                self.check_inside(end, 0, b) and self.check_inside(start, 0, b) and \
                self.check_inside(end, 2, b) and self.check_inside(start, 2, b):
                valid = True
            if (end[2] - self.blocks[b,2])*(start[2] - self.blocks[b,2]) <= 0 and \
                self.check_inside(end, 0, b) and self.check_inside(start, 0, b) and \
                self.check_inside(end, 1, b) and self.check_inside(start, 1, b):
                valid = True
        return valid
    
    def isInBoundary(self,newrp):
        if( newrp[0] < self.boundary[0,0] or newrp[0] > self.boundary[0,3] or \
           newrp[1] < self.boundary[0,1] or newrp[1] > self.boundary[0,4] or \
           newrp[2] < self.boundary[0,2] or newrp[2] > self.boundary[0,5] ):
            return False
        return True
    
    def isCollisionFree(self,newrp):
        valid = True
        for i in range(self.blocks.shape[0]):
            if( newrp[0] > self.blocks[i,0] and newrp[0] < self.blocks[i,3] and\
               newrp[1] > self.blocks[i,1] and newrp[1] < self.blocks[i,4] and\
               newrp[2] > self.blocks[i,2] and newrp[2] < self.blocks[i,5] ):
                valid = False
                break
        return valid
    
    def isVisited(self, point):
        for i in self.visited:
            if sum((i-point)**2) <= 0.1:
                return True
        return False
    
    def Astar(self,point,goal):
        newrobotpos = np.copy(point)
        numofdirs = 26
        [dX,dY,dZ] = np.meshgrid([-1,0,1],[-1,0,1],[-1,0,1])
        
        #27 different next step directions in the size of (3*27) 
        dR = np.vstack((dX.flatten(),dY.flatten(),dZ.flatten()))
        
        #Delete the (0,0,0) directions because it means the robot doesn't move
        dR = np.delete(dR,13,axis=1)
        
        #make every movement as the distance of 0.5 
        dR = dR / np.sqrt(np.sum(dR**2,axis=0)) / 2.0

        #myGraph is a dictionary with key: tuple(point) and value: (g,h)
        #open_list: (g+h, counter, point)
        #close_list: tuple(newrobotpos)

        if tuple(newrobotpos) not in self.myGraph:
            self.myGraph[tuple(newrobotpos)] = (self.g,self.computeHeuristic(newrobotpos,goal))
            
        for k in range(numofdirs):
            self.counter += 1

            #Go through the direction
            newrp = newrobotpos + dR[:,k]
            
            #Check if the newrp is valid
            if( newrp[0] < self.boundary[0,0] or newrp[0] > self.boundary[0,3] or \
               newrp[1] < self.boundary[0,1] or newrp[1] > self.boundary[0,4] or \
               newrp[2] < self.boundary[0,2] or newrp[2] > self.boundary[0,5] ):
                continue
            valid = True
            for i in range(self.blocks.shape[0]):
                if( newrp[0] > self.blocks[i,0] and newrp[0] < self.blocks[i,3] and\
                   newrp[1] > self.blocks[i,1] and newrp[1] < self.blocks[i,4] and\
                   newrp[2] > self.blocks[i,2] and newrp[2] < self.blocks[i,5] ):
                    valid = False
                    break
            if not valid:
                continue
            
            #Check if new point is in the Close List 
            if tuple(newrp) not in self.close_list:
                
                #Compute the cost of the moving direction
                cij = self.computeCost(dR[:,k])
            
                #Compute the heuristic of the new position
                h = self.computeHeuristic(newrp,goal)
            
                #Compute gj
                if tuple(newrp) not in self.myGraph: 
                    gj = self.g + cij
                    self.myGraph[tuple(newrp)] = (gj, h)
                    self.open_list.append((gj + h,gj,self.counter, newrp))
                
                elif tuple(newrp) in self.myGraph: 
                    gj = self.myGraph[tuple(newrp)][0]
                    if gj > (self.g + cij):
                        gj = self.g + cij
                        self.myGraph[tuple(newrp)] = (gj,h) 
                        self.open_list.append((gj + h,gj, self.counter, newrp))
        switch = 0 
        tmp = []
        while switch == 0:
            node = heapq.heappop(self.open_list)
            if sum((node[3] - newrobotpos)**2) > 1:
                tmp.append(node)
                continue
            else:
                switch = 1 

        for i in tmp:
            heapq.heappush(self.open_list,i)
        
        newrobotpos = node[3]
    
        self.close_list.append(tuple(newrobotpos))
        self.g = node[1]
        
        return newrobotpos
    
    def RTAAstar(self,start,goal):
        newrobotpos = np.copy(start)
        #myGraph is a dictionary with key: tuple(point) and value: (g,h)
        if tuple(newrobotpos) not in self.myGraph:
            self.myGraph[tuple(newrobotpos)] = [0,self.computeHeuristic(newrobotpos,goal)]
            
        numofdirs = 26
        #方向
        [dX,dY,dZ] = np.meshgrid([-1,0,1],[-1,0,1],[-1,0,1])
        #meshgrid什么意思造一个三维矩阵
        #27 different next step directions in the size of (3*27)
        dR = np.vstack((dX.flatten(),dY.flatten(),dZ.flatten()))

        #Delete the (0,0,0) directions because it means the robot doesn't move
        dR = np.delete(dR,13,axis=1)
        
        #make every movement as the distance of 0.5 
        dR = dR / 2
#        dR = dR / np.sqrt(np.sum(dR**2,axis=0)) / 2.0

        #open_list: (g+h, counter, point)
        #close_list: tuple(newrobotpos)

        expand_num = 0 
        self.close_list = []
        self.open_list = []
        #expand_num表示look_ahead
        while expand_num < self.expand_num and sum((start-goal)**2) > 0.1 :
        #如果扩展的点不到20并且距离终点还有很远
            for k in range(numofdirs):
                self.counter += 1
                #Go through the direction
                newrp = start + dR[:,k]
                
                #Check if the newrp is valid
                if not self.isInBoundary(newrp):
                    continue
                #如果在边界外或者是障碍，那么直接跳过
                if not self.isCollisionFree(newrp):
                    continue

                if self.segmentCollision(start,newrp):
                    continue
                
                #Check if new point is in the Close List 
                if tuple(newrp) in self.close_list:
                    continue
                #在closedset里面也跳过，表示已经搜索过了
                #Compute the cost of the moving direction
                cij = self.computeCost(dR[:,k])
            
                #Compute the heuristic of the new position
                hj = self.computeHeuristic(newrp,goal)
                
                gi = self.myGraph[tuple(newrobotpos)][0]

                #Compute gj
                if tuple(newrp) not in self.myGraph: 
                    gj = 1000000
                
                else: #tuple(newrp) in self.myGraph:
                    gj = self.myGraph[tuple(newrp)][0]
                    
                    
                if gj > (gi + cij):
                    gj = gi + cij
                    self.myGraph[tuple(newrp)] = (gj,hj)
                    
                heapq.heappush(self.open_list,(gj + self.epsilon*hj, self.counter, newrp))
            
            #Add up expand number of node
            expand_num += 1         
            node = heapq.heappop(self.open_list)
            fstar = node[0]
            start = node[2]
            self.close_list.append(tuple(start))
        
        #Update heuristic
        for j in self.close_list: 
            g = self.myGraph[j][0]
            updated_h = fstar - g
            self.myGraph[j] = (g,updated_h)
            
        point_list = []
        start = newrobotpos.copy()
        self.counter = 0 
        
        for k in range(numofdirs):
            self.counter += 1 
            #Go through the direction
            newrp = start + dR[:,k]
            
            #Check if the newrp is valid
            if not self.isInBoundary(newrp):
                continue
                
            if not self.isCollisionFree(newrp):
                continue
        
            if self.segmentCollision(start, newrp):
                continue
            
            if self.isVisited(newrp):
                continue
            
            h = self.myGraph[tuple(newrp)][1]
            g = self.myGraph[tuple(newrp)][0]
            
            heapq.heappush(point_list,(g + self.epsilon*h, self.counter, newrp))
            
        while point_list == []: 
            for k in range(numofdirs):
                self.counter += 1 
                #Go through the direction
                newrp = start + dR[:,k]
            
                #Check if the newrp is valid
                if not self.isInBoundary(newrp):
                    continue
                
                if not self.isCollisionFree(newrp):
                    continue
        
                if self.segmentCollision(start, newrp):
                    continue
                
                h = self.myGraph[tuple(newrp)][1]
                g = self.myGraph[tuple(newrp)][0]
            
                heapq.heappush(point_list,(g + self.epsilon*h, self.counter, newrp))
            
            
        new_node = heapq.heappop(point_list)
        newrobotpos = new_node[2]
        self.visited.append(newrobotpos)
        return newrobotpos
    
    def generateRandomPoint(self):
        return [np.random.uniform(self.boundary[0,3], self.boundary[0,0]), np.random.uniform(self.boundary[0,4], self.boundary[0,1]), np.random.uniform(self.boundary[0,5], self.boundary[0,2]) ]    
 
        
    def computeDist(self,start,goal):
        dis = np.sqrt((start[0]-goal[0])**2 + (start[1]-goal[1])**2 + (start[2]-goal[2])**2)
        return dis
    
    def getNearestNode(self, random_point, node_list):
        dist = []
        for i in node_list:
            temp = self.computeDist(i, random_point)
            dist.append(temp)
        index = dist.index(min(dist))
        return index
    
    def getSteeringNode(self,random_point, node): 
        #if the distance is greater than 1, we need to find a new random point on the segment
        while self.computeDist(node,random_point) > 1: 
            random_point[0] = node[0]*0.5 + random_point[0]*0.5
            random_point[1] = node[1]*0.5 + random_point[1]*0.5
            random_point[2] = node[2]*0.5 + random_point[2]*0.5
        steer_new = [random_point[0],random_point[1],random_point[2]]
        return steer_new
    
    def point2string(self,point):
        return " ".join(str(x) for x in point)
    
    def RRT(self,start,goal):
        #Put start in the node list (point)
        node_list = []
        node_list.append(start)
        parent_list = {}
        parent_list[self.point2string(start)] = start
        
        count = 0
        
        while self.point2string(goal) not in parent_list:
            if np.random.rand() > 0.1:
                random_point = self.generateRandomPoint()
            else:
                random_point = np.copy(goal)
                
            count+=1
            print(count)
        
            #Get nearest point from the node list 
            nearest_point_index = self.getNearestNode(random_point, node_list)
            nearest_point = node_list[nearest_point_index]
            
            #Can only move for dist = 1 
            steering_point = self.getSteeringNode(random_point, nearest_point)
        
            #Check if the newrp is valid
            if( steering_point[0] < self.boundary[0,0] or steering_point[0] > self.boundary[0,3] or \
               steering_point[1] < self.boundary[0,1] or steering_point[1] > self.boundary[0,4] or \
               steering_point[2] < self.boundary[0,2] or steering_point[2] > self.boundary[0,5] ):
                continue
            valid = True
            for i in range(self.blocks.shape[0]):
                if( steering_point[0] > self.blocks[i,0] and steering_point[0] < self.blocks[i,3] and\
                   steering_point[1] > self.blocks[i,1] and steering_point[1] < self.blocks[i,4] and\
                   steering_point[2] > self.blocks[i,2] and steering_point[2] < self.blocks[i,5] ):
                    valid = False
                    break
            if self.segmentCollision(steering_point, nearest_point):
                valid = False
                
            if not valid:
                continue
            
            node_list.append(steering_point)
            parent_list[self.point2string(steering_point)] = nearest_point

        #Extract the path
        #from goal to extract the path to start point
        path = [np.copy(goal)]
        v = np.copy(goal)

        while tuple(start) != tuple(v):
            v = parent_list[self.point2string(v)]
            path.append(np.array(v))
            
        #return the reverse path from start to goal
        return path[::-1]
    
        
        
        
                
                
                    
                    
                
                
                 


