package adtpackage;

import generalpackage.Printable;

/**
 *
 * @author Zbyněk Stara
 */
public class List implements Printable {
    private class Node <T extends Printable> {
        private T data = null;
        private Node next = null;

        private Node(T data) {
            this.data = data;
        }
    }

    private Node head = null;
    private int size = 0;

    public List() {
        head = null;
    }

    public boolean isEmpty() {
        return (head == null);
    }

    public int size() { // size is 1 when there is 1 element – which the 0th element
        return size;
    }

    public <T extends Printable> void insertAtFront(T data) {
        Node newNode = new Node(data);
        newNode.next = head;
        head = newNode;
        size += 1;
    }

    public <T extends Printable> void insertAtRear(T data) {
        if (isEmpty()) {
            head = new Node(data);
        } else {
            Node current = head;
            while (current.next != null) {
                current = current.next;
            }
            current.next = new Node(data);
        }
        size += 1;
    }

    public <T extends Printable> void insertAsNode(T data, int node) {
        // insert the data to be the node at specified index
        // whatever might have been there is moved back
        if (node == 0) {
            insertAtFront(data);
        } else if((!isEmpty()) && (node < size)) {
            Node current = head;
            for (int i = 1; i < node; i++) {
                current = current.next;
            }

            Node temp = current.next;
            current.next = new Node(data);
            current.next.next = temp;

            size += 1;
        } else {
            insertAtRear(data);
        }
    }
    
    public <T extends Printable> int search(T data) {
        if (isEmpty()) {
            return -1;
        } else {
            Node current = head;

            int counter = 0;
            while (current.data != data && current.next != null) {
                counter++;
                current = current.next;
            }

            if (current.data == data) return counter;
            else return -1;
        }
    }

    public Object removeFirst() {
        if (isEmpty()) {
            return null;
        }
        Node first = head;
        head = head.next;
        size -= 1;
        return first.data;
    }

    public Object removeLast() {
        if (isEmpty()) {
            return null;
        }
        Node current = head;
        if (current.next == null) {
            head = null;
            return current.data;
        }
        Node previous = null;
        while (current.next != null) {
            previous = current;
            current = current.next;
        }
        previous.next = null;
        size -= 1;
        return current.data;
    }

    public Object removeNode(int node) {
        if ((!isEmpty()) && (node < size)) {
            Node current = head;
            if (node == 0) {
                return removeFirst();
            }
            current = current.next;
            Node previous = head;

            for (int i = 1; i < node; i++) {
                current = current.next;
                previous = previous.next;
            }

            Node temp = current;
            previous.next = current.next;
            size -= 1;
            return temp.data;
        } else {
            return null;
        }
    }

    public Object getNodeData(int node) {
        if ((!isEmpty()) && (node < size)) {
            Node current = head;
            for (int i = 0; i < node; i++) {
                current = current.next;
            }
            return current.data;
        } else {
            return null;
        }
    }
    
    public Object getLastNodeData() {
        return getNodeData(size-1);
    }

    @Override public String print() {
        return toString();
    }

    @Override public String toString() {
        if (!isEmpty()) {
            String printedNodes = "";
            for (int i = 0; i < (size - 1); i++) {
                printedNodes = (printedNodes+"{"+printNodeData(i)+"}, ");
            }
            printedNodes = (printedNodes+"{"+printNodeData(size - 1)+"}");
            return ("List size: "+size+"; List contents: ["+printedNodes+"]");
        } else {
            return ("List is empty");
        }
    }

    private <T extends Printable> String printNodeData(int node) {
        if (getNodeData(node) == null) return null;
        else return ((T) getNodeData(node)).print();
    }
}
