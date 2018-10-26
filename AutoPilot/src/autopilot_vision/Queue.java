package autopilot_vision;

import java.util.Iterator;
import java.util.NoSuchElementException;

/**
 * A class of first-in-first-out (FIFO) queues of generic items.
 * 
 * @author 	Team Saffier
 * @version 	1.0
 * @note 	Taken from https://algs4.cs.princeton.edu/13stacks/Queue.java.html
 */
public class Queue<Item> implements Iterable<Item> {

    /**
     * A helper class for linked lists.
     * 
     * @author 	Team Saffier
     * @version 	1.0
     */
    private static class Node<Item> {
        private Item item;
        private Node<Item> next;
    }

    /**
     * Initializes an empty queue.
     */
    public Queue() {
        first = null;
        last  = null;
        n = 0;
    }

    /**
	 * The first node in this queue.
	 */
    private Node<Item> first;
    
    /**
     * The last node in this queue.
     */
    private Node<Item> last;
    
    /**
     * The number of elements in this queue.
     */
    private int n;
    
    /**
     * Returns true if this queue is empty.
     */
    public boolean isEmpty() {
        return first == null;
    }

    /**
     * Returns the number of items in this queue.
     */
    public int size() {
        return n;
    }

    /**
     * Returns the item least recently added to this queue.
     *
     * @return 	The item least recently added to this queue.
     * @throws 	NoSuchElementException
     * 			This queue is empty.
     */
    public Item peek() {
        if (isEmpty()) 
        		throw new NoSuchElementException("Queue underflow.");
        return first.item;
    }

    /**
     * Adds the given item to this queue.
     *
     * @param  	item 
     * 			The item to add to this queue.
     */
    public void enqueue(Item item) {
        Node<Item> oldlast = last;
        last = new Node<Item>();
        last.item = item;
        last.next = null;
        if (isEmpty()) 
        		first = last;
        else           
        		oldlast.next = last;
        n++;
    }

    /**
     * Removes and returns the item on this queue that was least recently added.
     *
     * @return	The item on this queue that was least recently added.
     * @throws 	NoSuchElementException 
     * 			If this queue is empty.
     * 			| this.isEmpty()
     */
    public Item dequeue() {
        if (isEmpty()) 
        		throw new NoSuchElementException("Queue underflow");
        Item item = first.item;
        first = first.next;
        n--;
        if (isEmpty()) 
        		last = null;   // to avoid loitering
        return item;
    }

    /**
     * Returns a string representation of this queue.
     *
     * @return The sequence of items in FIFO order, separated by spaces.
     */
    public String toString() {
        StringBuilder s = new StringBuilder();
        for (Item item : this) {
            s.append(item);
            s.append(' ');
        }
        return s.toString();
    } 

    /**
     * Returns an iterator that iterates over the items in this queue in FIFO order.
     *
     * @return An iterator that iterates over the items in this queue in FIFO order
     */
    public Iterator<Item> iterator()  {
        return new ListIterator<Item>(first);  
    }

    /**
     * A helper class of list iterators. Removal of elements is not implemented.
     * 
     * @author 	Team Saffier
     * @version 	1.0
     */
    @SuppressWarnings("hiding")
	private class ListIterator<Item> implements Iterator<Item> {
    	
        /**
         * Initializes this new list iterator with given first item.
         * 
         * @param 	first
         * 			The first item to iterate over.
         */
        public ListIterator(Node<Item> first) {
            current = first;
        }
        
        /**
         * The current item in this iterator.
         */
        private Node<Item> current;
        
        /**
         * Returns whether or not this iterator has any items left to iterate over.
         */
        public boolean hasNext()  { 
        		return current != null;                     
        	}
        
        /**
         * Removes the last item that was iterated over (not implemented).
         */
        public void remove() { 
        		throw new UnsupportedOperationException();  
        }
        
        /**
         * Returns the next item in this iterator.
         * 
         * @throws 	NoSuchElementException
         * 			If there is no next element.
         * 			| !hasNext()
         */
        public Item next() {
            if (!hasNext()) 
            		throw new NoSuchElementException();
            Item item = current.item;
            current = current.next; 
            return item;
        }
        
    }
    
}