package autopilot_vision;

/**
 * A class of hash tables using separate chaining.
 * 
 * @author 	Team Saffier
 * @version 	1.0
 * @note 	Adapted from https://algs4.cs.princeton.edu/34hash/SeparateChainingHashST.java.html
 */
public class HashMap<Key,Value> {
	
	/**
	 * A constant denoting the initial capacity for hash maps.
	 */
    private static final int INIT_CAPACITY = 10; 


    /**
     * Initializes an empty symbol table.
     */
    public HashMap() {
        this(INIT_CAPACITY);
    } 

    /**
     * Initializes an empty symbol table with m chains.
     * 
     * @param 	m 
     * 			The initial number of chains.
     */
    @SuppressWarnings("unchecked")
	public HashMap(int m) {
        this.m = m;
        st = (SymbolTable<Key, Value>[]) new SymbolTable[m];
        for (int i = 0; i < m; i++)
            st[i] = new SymbolTable<Key, Value>();
    }
    
    /**
     * The number of key-value pairs in this hash map.
     */
    private int n;
    
    /**
     * The size of this hash map.
     */
    private int m;
    
    /**
     * The array of linked-list symbol tables of this hash map.
     */
    private SymbolTable<Key, Value>[] st;

    /**
     * Resize this hash map to have the given number of chains,
     * 	rehashing all of the keys.
     * 
     * @param 	chains
     * 			The new number of chains for this hash map.
     */
    private void resize(int chains) {
    		HashMap<Key, Value> temp = new HashMap<Key, Value>(chains);
        for (int i = 0; i < m; i++) {
            for (Key key : st[i].keys()) {
                temp.put(key, st[i].get(key));
            }
        }
        this.m  = temp.m;
        this.n  = temp.n;
        this.st = temp.st;
    }

    /**
     * Turn the hash code of the given key into an array index.
     * 
     * @param 	key
     * 			The key whose has code is to be transmuted.
     * @return	A valid array index for this hash map.
     */
    private int hash(Key key) {
        return (key.hashCode() & 0x7fffffff) % m;
    } 

    /**
     * Returns the number of key-value pairs in this symbol table.
     */
    public int size() {
        return n;
    } 

    /**
     * Returns true if this symbol table is empty.
     *
     * @return True if and only f this symbol table is empty;
     *         | size() == 0
     */
    public boolean isEmpty() {
        return size() == 0;
    }

    /**
     * Returns true if this symbol table contains the specified key.
     *
     * @param  	key 
     * 			The key to look for.
     * @return 	True if and only if this symbol table contains the given key;
     * @throws 	IllegalArgumentException
     * 			The given key is null.
     * 			| key == null
     */
    public boolean contains(Key key) {
        if (key == null) 
        		throw new IllegalArgumentException("Argument to contains() is null.");
        return get(key) != null;
    } 

    /**
     * Returns the value associated with the specified key in this symbol table.
     *
     * @param  	key 
     * 			The key whose value is desired.
     * @return 	The value associated with key in the symbol table,
     *         	or null if there's no such value.
     * @throws 	IllegalArgumentException
     * 			The given key is null.
     * 			| key == null
     */
    public Value get(Key key) {
        if (key == null) throw new IllegalArgumentException("Argument to get() is null.");
        int i = hash(key);
        return st[i].get(key);
    } 

    /**
     * Inserts the specified key-value pair into the symbol table, overwriting the old 
     * value with the new value if the symbol table already contains the specified key.
     * Deletes the specified key (and its associated value) from this symbol table
     * if the specified value is null.
     *
     * @param  	key 
     * 			The key to insert.
     * @param  	val 
     * 			The value of the key that is to be inserted.
     * @throws 	IllegalArgumentException 
     * 			The given key is null.
     * 			| key == null
     */
    public void put(Key key, Value val) {
        if (key == null) 
        		throw new IllegalArgumentException("First argument to put() is null.");
        if (val == null) {
            delete(key);
            return;
        }
        if (n >= 10*m) 
        		resize(2*m);
        int i = hash(key);
        if (!st[i].contains(key)) 
        		n++;
        st[i].put(key, val);
    } 

    /**
     * Removes the specified key and its associated value from this symbol table     
     * (if the key is in this symbol table).    
     *
     * @param  	key 
     * 			The key that is to be removed.
     * @throws	IllegalArgumentException
     * 			The given key is null.
     * 			| key == null
     */
    public void delete(Key key) {
        if (key == null) 
        		throw new IllegalArgumentException("argument to delete() is null");
        int i = hash(key);
        if (st[i].contains(key)) 
        		n--;
        st[i].delete(key);
        if (m > INIT_CAPACITY && n <= 2*m) 
        		resize(m/2);
    } 

    /**
     * Return the keys in this hash map as an iterable.
     */
    public Iterable<Key> keys() {
        Queue<Key> queue = new Queue<Key>();
        for (int i = 0; i < m; i++) {
            for (Key key : st[i].keys())
                queue.enqueue(key);
        }
        return queue;
    } 
    
}