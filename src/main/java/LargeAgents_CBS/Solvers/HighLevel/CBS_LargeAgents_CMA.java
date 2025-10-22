package LargeAgents_CBS.Solvers.HighLevel;


import BasicCBS.Instances.Agent;
import BasicCBS.Instances.MAPF_Instance;
import BasicCBS.Solvers.AStar.DistanceTableFromStart;
import BasicCBS.Solvers.AStar.SingleAgentAStar_Solver;
import BasicCBS.Solvers.ConstraintsAndConflicts.ConflictManagement.I_ConflictManager;
import BasicCBS.Solvers.ConstraintsAndConflicts.Constraint.Constraint;
import BasicCBS.Solvers.ConstraintsAndConflicts.Constraint.ConstraintSet;
import BasicCBS.Solvers.Move;
import BasicCBS.Solvers.RunParameters;
import BasicCBS.Solvers.SingleAgentPlan;
import BasicCBS.Solvers.Solution;
import LargeAgents_CBS.Solvers.Constraints.ConstraintSet_LargeAgents;
import LargeAgents_CBS.Solvers.LowLevel.DistanceTableFromStart_LargeAgents;
import LargeAgents_CBS.Solvers.LowLevel.DistanceTableHeuristic_LargeAgents;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;


public class CBS_LargeAgents_CMA extends CBS_LargeAgents{

    public DistanceTableFromStart distanceTable;

    @Override
    protected void init(MAPF_Instance instance, RunParameters runParameters) {
        super.init(instance, runParameters);
    }

    protected void initCBS(MAPF_Instance instance, RunParameters parameters){
        if(instance == null || parameters == null){throw new IllegalArgumentException();}

        this.initialConstraints = Objects.requireNonNullElseGet(parameters.constraints, ConstraintSet::new);
        this.currentConstraints = new ConstraintSet_LargeAgents();
        this.generatedNodes = 0;
        this.expandedNodes = 0;
        this.instance = instance;
        this.aStarHeuristic = this.lowLevelSolver instanceof SingleAgentAStar_Solver ?
                new DistanceTableHeuristic_LargeAgents(new ArrayList<>(this.instance.agents), this.instance.map) :
                null;
        this.distanceTable = this.lowLevelSolver instanceof SingleAgentAStar_Solver ?
                new DistanceTableFromStart_LargeAgents(new ArrayList<>(this.instance.agents), this.instance.map) :
                null;
    }

    @Override
    protected Solution runAlgorithm(MAPF_Instance instance, RunParameters parameters) {

        super.initOpen(Objects.requireNonNullElseGet(parameters.constraints, ConstraintSet::new));
        CBS_Node goal = mainLoop();
        return super.solutionFromGoal(goal);
    }

    private CBS_Node mainLoop() {
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

        for(int i = 0; i < instance.agents.size(); i++) {
            if(i == constraintLeft.agent.iD || i == constraintRight.agent.iD){
                continue;
            }
            ids.add(i);
        }

        // Collections.shuffle(ids); //if you want random element in tie-breaking group split
        int splitIndex = ids.size() / 2;
        List<Integer> group1 = new ArrayList<>(ids.subList(0, splitIndex));
        List<Integer> group2 = new ArrayList<>(ids.subList(splitIndex, ids.size()));

        group1.add(constraintLeft.agent.iD);
        group2.add(constraintRight.agent.iD);

        ConstraintSet constraintSetLeft = new ConstraintSet();
        if (!super.isConstraintOnStartPosition(constraintLeft)) {
            for (Integer id : group1) {
                Constraint newConstraint = new Constraint(instance.agents.get(id), constraintLeft.time, constraintLeft.prevLocation, constraintLeft.location);
                constraintSetLeft.add(newConstraint);
            }
            CBS_Node leftChild = generateNode(node, constraintLeft, constraintSetLeft, true, group1);
            node.setLeftChild(leftChild);
        }

        ConstraintSet constraintSetRight = new ConstraintSet();
        if (!super.isConstraintOnStartPosition(constraintRight)) {
            for (Integer id : group2) {
                Constraint newConstraint = new Constraint(instance.agents.get(id), constraintRight.time, constraintRight.prevLocation, constraintRight.location);
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
        Solution solution = parent.getSolution();

        if (copyDatastructures) {
            solution = new Solution(solution);
        }

        boolean anyReplanned = false;
        for (Integer agentId : group) {
            Agent currentAgent = instance.agents.get(agentId);
            SingleAgentPlan currentAgentPlan = solution.getPlanFor(currentAgent);

            if (currentAgentPlan.getEndTime() >= constraint.time) {
                Move currentAgentMove = currentAgentPlan.moveAt(constraint.time);
                if (currentAgentMove.currLocation.intersectsWith(constraint.location)) {
                    solution.putPlan(new SingleAgentPlan(currentAgent));

                    Solution agentSolution = super.solveSubproblem(currentAgent, solution, buildConstraintSet(parent, constraintSet));

                    if (agentSolution == null) {
                        return null; //probably a timeout
                    }
                    solution.putPlan(agentSolution.getPlanFor(currentAgent));
                    anyReplanned = true;
                }
            }
            if (currentAgentPlan.getEndTime() < constraint.time && currentAgentPlan.moveAt(currentAgentPlan.getEndTime()).currLocation.intersectsWith(constraint.location)) {
                solution.putPlan(new SingleAgentPlan(currentAgent));

                Solution agentSolution = super.solveSubproblem(currentAgent, solution, buildConstraintSet(parent, constraintSet));

                if (agentSolution == null) {
                    return null; //probably a timeout
                }
                solution.putPlan(agentSolution.getPlanFor(currentAgent));
                anyReplanned = true;
            }

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
