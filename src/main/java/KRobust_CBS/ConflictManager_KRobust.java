package KRobust_CBS;

import BasicCBS.Instances.Agent;
import BasicCBS.Instances.Maps.I_Location;
import BasicCBS.Solvers.ConstraintsAndConflicts.ConflictManagement.ConflictManager;
import BasicCBS.Solvers.ConstraintsAndConflicts.ConflictManagement.ConflictSelectionStrategy;
import BasicCBS.Solvers.ConstraintsAndConflicts.ConflictManagement.DataStructures.AgentAtGoal;
import BasicCBS.Solvers.ConstraintsAndConflicts.ConflictManagement.DataStructures.TimeLocation;
import BasicCBS.Solvers.ConstraintsAndConflicts.VertexConflict;
import BasicCBS.Solvers.Move;
import BasicCBS.Solvers.SingleAgentPlan;

import javax.xml.stream.XMLInputFactory;
import java.util.HashSet;
import java.util.Set;

public class ConflictManager_KRobust extends ConflictManager {


    public ConflictManager_KRobust(ConflictSelectionStrategy conflictSelectionStrategy){
        super(conflictSelectionStrategy);
    }

    protected void addAgentNewPlan(SingleAgentPlan singleAgentPlan) {

        if ( singleAgentPlan == null ){ return; }

        int k = 0;
        if( singleAgentPlan.agent instanceof  RobustAgent){
            k = ((RobustAgent)singleAgentPlan.agent).k;
        }


        int agentFirstMoveTime = singleAgentPlan.getFirstMoveTime();
        int goalTime = singleAgentPlan.getEndTime();
        I_Location goalLocation = singleAgentPlan.moveAt(goalTime).currLocation;



//        /*  Check for conflicts and Add timeLocations */
//        for (int time = agentFirstMoveTime; time <= goalTime; time++) {
//            I_Location location = singleAgentPlan.moveAt(time).prevLocation;
//            int startTime = Math.max(1, time - k);  // when they arrive at this location
//            for (int kTime = startTime; kTime <= time; kTime++) {
//            //for (int kTime = time; kTime >= Math.max(1, time - k); kTime--) {
//                // Move's from location is 'prevLocation' , therefor timeLocation is time - 1
//                TimeLocation timeLocation = new TimeLocation(kTime - 1, location);
//
//                this.checkAddConflictsByTimeLocation(timeLocation, singleAgentPlan); // Checks for conflicts
//                this.timeLocationTables.addTimeLocation(timeLocation, singleAgentPlan);
//            }
//        }

        for (int time = agentFirstMoveTime; time <= goalTime; time++) {
            I_Location location = singleAgentPlan.moveAt(time).prevLocation;
            TimeLocation minTimeLocation = new TimeLocation(time - 1, location);

            // when they arrive at this location
            for (int kTime = Math.max(1, time - k); kTime <= time; kTime++) {
                //for (int kTime = time; kTime >= Math.max(1, time - k); kTime--) {
                // Move's from location is 'prevLocation' , therefor timeLocation is time - 1
                TimeLocation timeLocation = new TimeLocation(kTime - 1, location);

                this.checkAddConflictsByTimeLocation(timeLocation, singleAgentPlan); // Checks for conflicts
                this.timeLocationTables.addTimeLocation(timeLocation, singleAgentPlan);
            }
        }

        for (int time = Math.max(1, goalTime - k); time <= goalTime; time++) {
            TimeLocation timeLocation = new TimeLocation(time, goalLocation);
            this.checkAddConflictsByTimeLocation(timeLocation, singleAgentPlan); // Checks for conflicts
            this.timeLocationTables.addTimeLocation(timeLocation, singleAgentPlan);
        }

        // Checks for conflicts and add if exists. Adds the goal's timeLocation
        this.manageGoalLocationFromPlan(goalTime, singleAgentPlan);
    }

    protected void checkAddConflictsByTimeLocation(TimeLocation timeLocation, TimeLocation minTimeLocation, SingleAgentPlan singleAgentPlan) {

        Set<Agent> agentsAtTimeLocation = super.timeLocationTables.getAgentsAtTimeLocation(timeLocation);
        super.addVertexConflicts(minTimeLocation, singleAgentPlan.agent, agentsAtTimeLocation);

        /*  = Check conflicts with agents at their goal =    */
        AgentAtGoal agentAtGoal = super.timeLocationTables.getAgentAtGoalTime(timeLocation.location);
        if( agentAtGoal != null ){
            if ( timeLocation.time >= agentAtGoal.time ){
                // Adds a Vertex conflict if time at location is greater than another agent time at goal
                super.addVertexConflicts(minTimeLocation, singleAgentPlan.agent, new HashSet<>(){{add(agentAtGoal.agent);}});
            }
        }

        /*      = Check for swapping conflicts =     */
        super.checkAddSwappingConflicts(minTimeLocation.time, singleAgentPlan);
    }

    protected void manageGoalLocationFromPlan(int goalTime, SingleAgentPlan singleAgentPlan) {

        I_Location goalLocation = singleAgentPlan.moveAt(goalTime).currLocation;
        TimeLocation goalTimeLocation = new TimeLocation(goalTime, goalLocation);

        /*  = Check if this agentAtGoal conflicts with other agents =   */
        this.checkAddSwappingConflicts(goalTime, singleAgentPlan);
        this.checkAddVertexConflictsWithGoal(goalTimeLocation, singleAgentPlan);


        /*  = Add goal timeLocation =  */
        this.timeLocationTables.addGoalTimeLocation(goalTimeLocation, singleAgentPlan);
    }


    protected VertexConflict createVertexConflict(Agent agent1, Agent agent2, TimeLocation timeLocation){
        return new VertexConflict_KRobust(agent1, agent2, timeLocation);
    }


}
