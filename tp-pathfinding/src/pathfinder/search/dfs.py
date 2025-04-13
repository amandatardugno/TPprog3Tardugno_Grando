from ..models.grid import Grid
from ..models.frontier import StackFrontier
from ..models.solution import NoSolution, Solution
from ..models.node import Node


class DepthFirstSearch:
    @staticmethod
    def search(grid: Grid) -> Solution:
        """Find path between two points in a grid using Depth First Search

        Args:
            grid (Grid): Grid of points
            
        Returns:
            Solution: Solution found
        """
        # Initialize a node with the initial position
        node = Node("", grid.start, 0)

        # Check if the start is already the goal
        if node.state == grid.end:
            return Solution(node, reached={node.state: True})

        # Initialize the explored dictionary to be empty
        explored = {} 
        
        # Add the node to the explored dictionary
        explored[node.state] = True

        # Initialize the frontier with a stack
        frontier = StackFrontier()
        frontier.add(node)

        while not frontier.is_empty():
            # Remove the most recently added node (LIFO)
            node = frontier.remove()

            # Explore neighbors
            for action, new_state in grid.get_neighbours(node.state).items():
                if new_state not in explored:
                    # Create a new node
                    new_node = Node(
                        "", 
                        state=new_state,
                        cost=node.cost + grid.get_cost(new_state),
                        parent=node,
                        action=action
                    )

                    # Check if it's the goal
                    if new_state == grid.end:
                        return Solution(new_node, explored)

                    # Mark as explored and add to the frontier
                    explored[new_state] = True
                    frontier.add(new_node)

        # No solution found
        return NoSolution(explored)

