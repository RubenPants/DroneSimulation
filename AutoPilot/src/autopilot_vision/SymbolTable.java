package autopilot_vision;

/**
 * A class of symbol tables for sequential searching.
 * 
 * @author 	Team Saffier
 * @version 	1.0
 */
public class SymbolTable<Key, Value> {

    /**
     * Helper class for linked lists.
     * 
     * @author 	Team Saffier
     * @version 	1.0
     */
    private class Node {

        /**
         * Initialise this new node with given key, value and nex node.
         * @param 	key
         * 			The key for this new node.
         * @param 	val
         * 			The value for this new node.
         * @param 	next
         * 			The next node for this new node.
         */
        public Node(Key key, Value val, Node next)  {
            this.key  = key;
            this.val  = val;
            this.next = next;
        }
        
        /**
		 * The key associated with this node.
		 */
	    private Key key;
	    
	    /**
		 * The value associated with this node.
		 */
	    private Value val;
	    
	    /**
		 * The next node linked to this node.
		 */
	    private Node next;
        
    }

    /**
     * Initializes an empty symbol table.
     */
    public SymbolTable() {
    		// 
    }

    /**
     * Returns the number of key-value pairs in this symbol table.
     */
    public int size() {
        return n;
    }

    /**
     * Returns whether or not this symbol table is empty.
     * 
     * @return	True if and only if this symbol table is empty.
     * 			| size() == 0
     */
    public boolean isEmpty() {
        return size() == 0;
    }
    
    /**
	 * The number of key-value pairs in this symbol table.
	 */
    private int n;
    
    /**
     * The first node in this symbol table.
     */
    private Node first;

    /**
     * Check whether or not this symbol table contains the given key.
     * 
     * @param 	key
     * 			The key to look for.
     * @return 	True if and only if this symbol table contains the given key,
     * 			false otherwise.
     * 			| get(key) != null
     */
    public boolean contains(Key key) {
        return get(key) != null;
    }

    /**
     * Returns the value associated with the given key.
     * 
     * @param 	key 
     * 			The key whose associated value is desired.
     * @return 	The value associated with the given key if the key is in the symbol table
     *     		or null if the key is not in the symbol table.
     */
    public Value get(Key key) {
        for (Node x = first; x != null; x = x.next) {
            if (key.equals(x.key))
                return x.val;
        }
        return null;
    }

    /**
     * Inserts the key-value pair into the symbol table, overwriting the old value
     * 	with the new value if the key is already in the symbol table.
     * If the value is null, this effectively deletes the key from the symbol table.
     * 
     * @param 	key 
     * 			The key to insert.
     * @param 	val 
     * 			The value of the key that iss to be inserted.
     */
    public void put(Key key, Value val) {
        if (val == null) {
            delete(key);
            return;
        }
        for (Node x = first; x != null; x = x.next) {
            if (key.equals(x.key)) {
                x.val = val;
                return;
            }
        }
        first = new Node(key, val, first);
        n++;
    }

    /**
     * Removes the key and associated value from the symbol table
     * 	(if the key is in the symbol table).
     * 
     * @param 	key 
     * 			The key to remove.
     */
    public void delete(Key key) {
        first = delete(first, key);
    }

    /**
     * Delete the key in the linked list beginning at node x.
     */
    private Node delete(Node x, Key key) {
        if (x == null) return null;
        if (key.equals(x.key)) {
            n--;
            return x.next;
        }
        x.next = delete(x.next, key);
        return x;
    }

    /**
     * Returns all keys in the symbol table as an iterable.
     * 
     * @return An iterable having the keys in this symbol table.
     */
    public Iterable<Key> keys()  {
        Queue<Key> queue = new Queue<Key>();
        for (Node x = first; x != null; x = x.next)
            queue.enqueue(x.key);
        return queue;
    }
    
}