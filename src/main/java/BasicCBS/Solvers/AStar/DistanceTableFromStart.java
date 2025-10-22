package BasicCBS.Solvers.AStar;

import BasicCBS.Instances.Agent;
import BasicCBS.Instances.Maps.*;
import BasicCBS.Instances.Maps.Coordinates.I_Coordinate;
import BasicCBS.Solvers.ConstraintsAndConflicts.Constraint.Constraint;

import java.util.*;

/**
 * A {@link AStarHeuristic} that uses a pre-calculated dictionary of distances from possible goal locations to every
 * accessible {@link I_Location location} to provide a perfectly tight heuristic.
 */
public class DistanceTableFromStart {
    // nicetohave avoid duplicates (when two agents have the same goal)

    // todo - protected
    protected Map<Agent, Map<I_Location, Integer>> distanceDictionaries;

    public Map<Agent, Map<I_Location, Integer>> getDistanceDictionaries() {
        return distanceDictionaries;
    }

    protected DistanceTableFromStart(){
        this.distanceDictionaries = new HashMap<>();
    }

    public DistanceTableFromStart(List<? extends Agent> agents, I_Map map) {

        this.distanceDictionaries = new HashMap<>();
        for (int i = 0; i < agents.size(); i++) {

            //this map will entered to distanceDictionaries for every agent
            Map<I_Location, Integer> mapForAgent = new HashMap<>();
            this.distanceDictionaries.put(agents.get(i), mapForAgent);
            LinkedList<I_Location> queue = new LinkedList<>();
            I_Coordinate i_coordinate = agents.get(i).source;
            I_Location mapCell = map.getMapCell(i_coordinate);

            //distance of a graphMapCell from itself
            this.distanceDictionaries.get(agents.get(i)).put(mapCell, 0);

            //all the neighbors of a graphMapCell
            List<I_Location> neighbors = mapCell.getNeighbors();
            for (int j = 0; j < neighbors.size(); j++) {
                queue.add(neighbors.get(j));
            }

            int distance = 1;
            int count = queue.size();

            while (!(queue.isEmpty())) {
                I_Location i_location = queue.remove(0);

                //if a graphMapCell didn't got a distance yet
                if (!(this.distanceDictionaries.get(agents.get(i)).containsKey(i_location))) {
                    this.distanceDictionaries.get(agents.get(i)).put(i_location, distance);

                    //add all the neighbors of the current graphMapCell to  the queue
                    List<I_Location> neighborsCell = i_location.getNeighbors();
                    queue.addAll(neighborsCell);
                }

                count--;
                if (count == 0) { //full level/round of neighbors is finish
                    distance++;
                    count = queue.size(); //start new level with distance plus one
                }
            }
        }
    }

    //@Override
    public float getH(Agent currentAgent, Constraint currentConstraint) {
        Map<I_Location, Integer> relevantDictionary = this.distanceDictionaries.get(currentAgent);
        if (relevantDictionary == null) {
            //System.err.println("No distance dictionary found for agent: " + currentAgent.iD);
            return Float.MAX_VALUE;
        }
        Integer distance = relevantDictionary.get(currentConstraint.location);
        if (distance == null) {
//            String locationType = (currentConstraint.location != null)
//                    ? currentConstraint.location.getClass().getSimpleName()
//                    : "null";
            //System.err.println("No distance found for location (" + locationType + ") for agent: " + currentAgent.iD);
            return Float.MAX_VALUE;
        }
        return distance.floatValue();
    }

}
