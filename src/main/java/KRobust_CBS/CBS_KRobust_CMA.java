package KRobust_CBS;

import BasicCBS.Instances.Agent;
import BasicCBS.Instances.MAPF_Instance;
import BasicCBS.Instances.Maps.Coordinates.I_Coordinate;
import BasicCBS.Solvers.*;
import BasicCBS.Solvers.AStar.DistanceTableAStarHeuristic;
import BasicCBS.Solvers.AStar.DistanceTableFromStart;
import BasicCBS.Solvers.AStar.SingleAgentAStar_Solver;
import BasicCBS.Solvers.ConstraintsAndConflicts.Constraint.Constraint;
import BasicCBS.Solvers.ConstraintsAndConflicts.Constraint.ConstraintSet;
import BasicCBS.Solvers.ConstraintsAndConflicts.ConflictManagement.I_ConflictManager;


import java.util.*;

public class CBS_KRobust_CMA extends CBS_KRobust {

    public DistanceTableFromStart distanceTable;

    @Override
    protected void init(MAPF_Instance instance, RunParameters runParameters) {
        super.init(instance, runParameters);
    }

    protected void initCBS(MAPF_Instance instance, RunParameters runParameters){
        this.initialConstraints = Objects.requireNonNullElseGet(runParameters.constraints, ConstraintSet::new);
        this.currentConstraints = new ConstraintSet_Robust();
        this.generatedNodes = 0;
        this.expandedNodes = 0;
        this.instance = instance;
        this.aStarHeuristic = this.lowLevelSolver instanceof SingleAgentAStar_Solver ?
                new DistanceTableAStarHeuristic(new ArrayList<>(this.instance.agents), this.instance.map) :
                null;
        this.distanceTable = this.lowLevelSolver instanceof SingleAgentAStar_Solver ?
                new DistanceTableFromStart(new ArrayList<>(this.instance.agents), this.instance.map) :
                null;
    }

    @Override
    protected Solution runAlgorithm(MAPF_Instance instance, RunParameters parameters) {
        super.initOpen(Objects.requireNonNullElseGet(parameters.constraints, ConstraintSet::new));
        CBS_Node goal = mainLoop_KRobust();
        return super.solutionFromGoal(goal);
    }

    private CBS_Node mainLoop_KRobust() {
        while(!openList.isEmpty() && !checkTimeout()){
            CBS_Node node = openList.poll();

            I_ConflictManager cat = getConflictAvoidanceTableFor(node);
            node.setSelectedConflict(cat.selectConflict());

            if(super.isGoal(node)) {
                return node;
            }
            else {
                expandNode(node);
            }
        }
        return null; //probably a timeout
    }

    private void expandNode(CBS_Node node) {
        this.expandedNodes++;

        Constraint[] constraints = node.getSelectedConflict().getPreventingConstraints();

        List<Integer> ids = new ArrayList<>();
        Constraint constraintLeft = constraints[0];
        Constraint constraintRight = constraints[1];

        int k = ((RobustAgent)instance.agents.get(0)).k;

        for(int i = 0; i < instance.agents.size(); i++) {
            if(i == constraintLeft.agent.iD || i == constraintRight.agent.iD){
                continue;
            }
            Agent currentAgent = instance.agents.get(i);
            // Use distanceTable for heuristic grouping
            float heuristic = distanceTable.getH(currentAgent, constraintLeft);
            if (heuristic <= constraintLeft.time + k) {
                ids.add(i);
            }
        }

        // Collections.shuffle(ids); // if you want random element in tie-breaking group split
        int splitIndex = ids.size() / 2;
        List<Integer> group1 = new ArrayList<>(ids.subList(0, splitIndex));
        List<Integer> group2 = new ArrayList<>(ids.subList(splitIndex, ids.size()));

        group1.add(constraintLeft.agent.iD);
        group2.add(constraintRight.agent.iD);

        // Create K-Robust constraints for the left child (group1)
        ConstraintSet constraintSetLeft = new ConstraintSet();

        if (!super.isConstraintOnStartPosition(constraintLeft)) {
            for (Integer id : group1) {
                Constraint newConstraint = new Constraint_Robust(instance.agents.get(id), constraintLeft.time, constraintLeft.prevLocation, constraintLeft.location);
                constraintSetLeft.add(newConstraint);
            }
            CBS_Node leftChild = generateNode(node, constraintLeft, constraintSetLeft, true, group1);
            node.setLeftChild(leftChild);
        }


        // Create K-Robust constraints for the right child (group2)
        ConstraintSet constraintSetRight = new ConstraintSet();

        if (!super.isConstraintOnStartPosition(constraintRight)) {
            for (Integer id : group2) {
                Constraint newConstraint = new Constraint_Robust(instance.agents.get(id), constraintRight.time, constraintRight.prevLocation, constraintRight.location);
                constraintSetRight.add(newConstraint);
            }
            CBS_Node rightChild = generateNode(node, constraintRight, constraintSetRight, true, group2);
            node.setRightChild(rightChild);
        }


        super.addToOpen(node.getLeftChild());
        super.addToOpen(node.getRightChild());
    }

    private CBS_Node generateNode(CBS_Node parent, Constraint constraint, ConstraintSet constraintSet, boolean copyDatastructures, List<Integer> group) {
        super.generatedNodes++;
        Solution solution = parent.getSolution(); // Copy parent solution
        int lowerBound = constraint.time;
        int upperBound = constraint.time;
        if (constraint instanceof Constraint_Robust) {
            Constraint_Robust constraintRobust = (Constraint_Robust) constraint;
            lowerBound = Math.max(1, constraintRobust.getLowerBound());
            upperBound = constraintRobust.getUpperBound();
        }
        if (copyDatastructures) {
            solution = new Solution(solution);
        }
        I_Coordinate constraintCoordinate = constraint.location.getCoordinate();
        for (Integer agentId : group) {
            Agent currentAgent = instance.agents.get(agentId);

            SingleAgentPlan currentAgentPlan = solution.getPlanFor(currentAgent);

            if (currentAgentPlan.getEndTime() < lowerBound && !currentAgent.target.equals(constraintCoordinate)) {
                continue;
            }
            if (currentAgentPlan.getEndTime() >= lowerBound) {
                int i = Math.min(upperBound, currentAgentPlan.getEndTime());
                for (; i >= lowerBound; i--) {
                    Move currentAgentMove = currentAgentPlan.moveAt(i);
                    if (currentAgentMove.currLocation == constraint.location) {
                        solution.putPlan(new SingleAgentPlan(currentAgent));

                        Solution agentSolution = super.solveSubproblem(currentAgent, solution, buildConstraintSet(parent, constraintSet));

                        if (agentSolution == null) {
                            return null; //probably a timeout
                        }
                        solution.putPlan(agentSolution.getPlanFor(currentAgent));

                        break;
                    }
                }
                continue;
            }

            solution.putPlan(new SingleAgentPlan(currentAgent));

            Solution agentSolution = super.solveSubproblem(currentAgent, solution, buildConstraintSet(parent, constraintSet));

            if (agentSolution == null) {
                return null; //probably a timeout
            }
            solution.putPlan(agentSolution.getPlanFor(currentAgent));

        }
        return new CBS_Node(solution, costFunction.solutionCost(solution, this), constraint, constraintSet, parent);
    }

    protected ConstraintSet buildConstraintSet(CBS_Node parentNode, ConstraintSet newConstraintSet) {
        super.currentConstraints.clear();
        ConstraintSet constraintSet = this.currentConstraints;
        constraintSet.addAll(initialConstraints);

        CBS_Node currentNode = parentNode;
        while (currentNode.getAddedConstraintSet() != null){

            constraintSet.addAll(currentNode.getAddedConstraintSet());
            currentNode = currentNode.getParent();
        }

        constraintSet.addAll(newConstraintSet);

        return constraintSet;
    }


    @Override
    protected void releaseMemory() {
        super.releaseMemory();
        this.distanceTable = null;
    }
}