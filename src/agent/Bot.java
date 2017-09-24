package agent;

import problem.ProblemSpec;

import java.io.*;
import java.util.List;

public class Bot {
    public static void main(String[] args) throws IOException {
        String inputFileName = args[0];
        String outputFileName = args[1];

        ProblemSpec problemSpec = new ProblemSpec();
        problemSpec.loadProblem(inputFileName);
        SearchAgent agent = new SearchAgent(problemSpec);

        /* Run program */
        agent.run();
        problemSpec.saveSolution(outputFileName);
    }
}
