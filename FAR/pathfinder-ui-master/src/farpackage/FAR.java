package farpackage;

import apackage.*;
import adtpackage.*;
import generalpackage.*;

/**
 *
 * @author Zbyněk Stara
 */
public class FAR {
    public FAR() {
    }

    public void resetAExtensions(Field field) {
        for (int i = 0; i < field.X_DIMENSION; i++) {
            for (int j = 0; j < field.Y_DIMENSION; j++) {
                field.getElement(i, j).setAExtension(new AExtension());
            }
        }
    }//重置A的扩展

    public List far(Element start, Element goal, Field field) {
        // returns path from start to goal, inclusively
        
        HashSet closedSet = new HashSet((field.X_DIMENSION - 2) *
                (field.Y_DIMENSION - 2)); // The set of nodes already evaluated.
        // ORDERED BY ELEMENT KEYS
        MinimalQueue openSet = new MinimalQueue(); // The set of tentative nodes to be evaluated, initially containing the start node
        // ORDERED BY F_SCORES

        double gScore = 0; // Cost from start along best known path.
        double hScore = field.manhattanDistanceBetween(start, goal); // manhattanDistanceBetween == right-angle heuristic cost estimate
        double fScore = gScore + hScore; // Estimated total cost from start to goal through y.

        start.getAExtension().setScores(gScore, hScore, fScore);

        openSet.enqueue(start, start.getAExtension().getFScore());

        //A*迭代
        while (!openSet.isEmpty()) {
            //Element current = openSetChooser(field, openSet, closedSet); // chooses the smallest from open set
            Element current = (Element) openSet.dequeue();
            List neighborList = current.getFARExtension().getAccessibleElements();

            if (current.isEqual(goal)) {
                return reconstructPath(start, goal);
            }//如果路径规划到了终点，那么直接构建路径

            while (!neighborList.isEmpty()) {
                Element currentNeighbor = (Element) neighborList.removeFirst();
                double tentativeGScore = -999;

                if ((currentNeighbor.isBoundary()) || (currentNeighbor.isObstacle())) {
                    continue;//碰到边界或者障碍跳过
                } else if (!field.isLegalMove(current, currentNeighbor)) {
                    continue;//碰到合法移动跳过
                } else if (closedSet.contains(currentNeighbor.getKey(field.Y_DIMENSION))) {
                    continue;//碰到已经搜索过的点，跳过
                } else {
                    tentativeGScore = current.getAExtension().getGScore() +
                            field.manhattanDistanceBetween(current, currentNeighbor);
                    //试验性的g分数
                    // to penalize non-straight paths - one of the requirements of FAR
                    // also penalizes initial paths (since current.cameFrom is null), but this happens across the board, so it's not a problem
                    if (!field.isStraightLine(current.getAExtension().getCameFrom(), current, currentNeighbor))
                        tentativeGScore += 0.1;
                    //如果当前点是直线的话，那么试验性G+0.1，非直线路径惩罚
                }

                if (openSet.search(currentNeighbor) == -1) { // if current neighbor is not in open set
                    //如果当前的邻居节点不在openlist要扩展的里面
                    currentNeighbor.getAExtension().setCameFrom(current);
                    //对当前邻居节点计算f，g，h
                    gScore = tentativeGScore;
                    hScore = field.manhattanDistanceBetween(currentNeighbor, goal);
                    fScore = gScore + hScore;

                    currentNeighbor.getAExtension().setScores(gScore, hScore, fScore);

                    openSet.enqueue(currentNeighbor, currentNeighbor.getAExtension().getFScore());
                } else if (tentativeGScore < currentNeighbor.getAExtension().getGScore()) {
                    // else if current neighbor is in openSet but new GScore is better
                    openSet.delete(currentNeighbor);

                    currentNeighbor.getAExtension().setCameFrom(current);

                    gScore = tentativeGScore;
                    hScore = currentNeighbor.getAExtension().getHScore(); // hScore is already set for this one
                    fScore = gScore + hScore;

                    currentNeighbor.getAExtension().setScores(gScore, hScore, fScore);

                    openSet.enqueue(currentNeighbor, currentNeighbor.getAExtension().getFScore());
                } else { // else if current neighbor is in openSet and new GScore is not better
                    // then nothing happens
                }
            }
            closedSet.add(current, current.getKey(field.Y_DIMENSION));
        }
        return null;
    }

    private List reconstructPath(Element startNode, Element currentNode) {
        List path;
        if (currentNode != startNode) { //came_from[current_node] is set {
            List tempPath = reconstructPath(startNode, currentNode.getAExtension().getCameFrom());
            path = tempPath;
            path.insertAtRear(currentNode);
        }
        else {
            path = new List();
            path.insertAtRear(currentNode);
        }
        return path;
    }
}
