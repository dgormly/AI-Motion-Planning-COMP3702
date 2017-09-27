package agent;

import problem.ASVConfig;

public class Edge {
    public ASVConfig cfg1;
    public ASVConfig cfg2;

    public Edge(ASVConfig cfg1, ASVConfig cfg2) {
        this.cfg1 = cfg1;
        this.cfg2 = cfg2;
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;

        Edge edge = (Edge) o;

        if (!cfg1.equals(edge.cfg1)) return false;
        return cfg2.equals(edge.cfg2);
    }

    @Override
    public int hashCode() {
        int result = cfg1.hashCode();
        result = 31 * result + cfg2.hashCode();
        return result;
    }
}