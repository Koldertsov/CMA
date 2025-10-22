package BasicCBS.Solvers.ConstraintsAndConflicts.ConflictManagement;

import BasicCBS.Solvers.ConstraintsAndConflicts.A_Conflict;

import java.util.Collection;
import java.util.Iterator;
import java.util.PriorityQueue;
import java.util.SortedSet;

/**
 * Selects a {@link A_Conflict} with the maximum time.
 */
public class MaxTimeConflictSelectionStrategy implements ConflictSelectionStrategy {
    @Override
    public A_Conflict selectConflict(Collection<A_Conflict> conflicts) {
        if(conflicts == null || conflicts.isEmpty()) {return null;}

        //if a sorted Collection, assume it is sorted by time, and select the last
        if(conflicts instanceof SortedSet){
            SortedSet<A_Conflict> sortedConflicts = ((SortedSet<A_Conflict>)conflicts);
            return sortedConflicts.last();
        }
        if(conflicts instanceof PriorityQueue){
            // Note: PriorityQueue.peek() returns the head based on the queue's ordering
            // If the queue is ordered for minimum time, this won't give the maximum
            // For proper max-time behavior with PriorityQueue, the queue should be
            // configured with a reverse comparator or we need to iterate through all elements
            PriorityQueue<A_Conflict> sortedConflicts = ((PriorityQueue<A_Conflict>)conflicts);

            // Iterate through all elements to find the maximum time conflict
            A_Conflict maxTimeConflict = null;
            for(A_Conflict conflict : sortedConflicts) {
                if(maxTimeConflict == null || conflict.time > maxTimeConflict.time) {
                    maxTimeConflict = conflict;
                }
            }
            return maxTimeConflict;
        }

        Iterator<A_Conflict> iter = conflicts.iterator();
        if(!iter.hasNext()){return null;} //might be empty (no conflicts)
        else{
            // find maximum
            A_Conflict maxTimeConflict = iter.next();
            while(iter.hasNext()){
                A_Conflict candidate = iter.next();
                if(maxTimeConflict.time < candidate.time) {maxTimeConflict = candidate;}
            }
            return maxTimeConflict;
        }
    }
}