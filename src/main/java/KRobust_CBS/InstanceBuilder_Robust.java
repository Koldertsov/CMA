package KRobust_CBS;

import BasicCBS.Instances.Agent;
import BasicCBS.Instances.InstanceBuilders.InstanceBuilder_MovingAI;

import BasicCBS.Instances.Maps.Coordinates.Coordinate_2D;

public class InstanceBuilder_Robust extends InstanceBuilder_MovingAI {


    protected final int INDEX_AGENT_ROBUST_VALUE = 5;
    private int k = 0;

    public InstanceBuilder_Robust(){ }

    public InstanceBuilder_Robust(int k){
        super();
        this.k = k;
    }

    @Override
    protected Agent buildSingleAgent(int id, String agentLine) {

        String[] splitLine = agentLine.split(this.SEPARATOR_SCENARIO);
        // Init coordinates
        int source_xValue = Integer.parseInt(splitLine[this.INDEX_AGENT_SOURCE_XVALUE]);
        int source_yValue = Integer.parseInt(splitLine[this.INDEX_AGENT_SOURCE_YVALUE]);
        Coordinate_2D source = new Coordinate_2D(source_xValue, source_yValue);
        int target_xValue = Integer.parseInt(splitLine[this.INDEX_AGENT_TARGET_XVALUE]);
        int target_yValue = Integer.parseInt(splitLine[this.INDEX_AGENT_TARGET_YVALUE]);
        Coordinate_2D target = new Coordinate_2D(target_xValue, target_yValue);

        Agent agent = new Agent(id, source, target);

        return this.createAgent(agent, this.k);
    }




    protected Agent createAgent(Agent agent, int k){
        return new RobustAgent(agent, k);
    }


}
