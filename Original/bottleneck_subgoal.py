import numpy as np
import robotic as ry
import networkx as nx
import matplotlib.pyplot as plt


def visualize_grid_graph(G, grid_resolution, filename="grid_graph.png"):
    pos = {node: node for node in G.nodes}  # Store node positions as their coordinates
    
    plt.figure(figsize=(8, 8))

    # Draw edges
    nx.draw_networkx_edges(G, pos, width=1.0, alpha=0.5, edge_color="gray")

    # Draw nodes as small dots instead of large circles
    x_vals, y_vals = zip(*pos.values())  # Extract x, y coordinates of nodes
    plt.scatter(x_vals, y_vals, s=10, color="blue")  # s=10 controls dot size

    plt.title("Grid Graph Visualization")
    plt.axis('equal')  # Keep the grid aspect ratio
    plt.grid(True, linestyle="--", linewidth=0.5)  # Optional: Add a light grid

    # Save the plot
    plt.savefig(filename, format='PNG', dpi=300)  # Higher dpi for clarity
    print(f"Graph saved as {filename}")

    plt.close()  # Close the plot to avoid displaying in interactive environments

def get_collidable_objects(C:ry.Config) -> list:
    collidable_objects = []
    entities = C.getCollidablePairs()
    objects = set(entities)

    for entity in objects:
        e = C.getFrame(entity)
        pos = e.getPosition()
        size = e.getSize()
        collidable_objects.append({'name': entity, 'position': pos, 'size': size})
    return collidable_objects

def define_grid(grid_resolution=0.1):
    # Use linspace to ensure the grid includes all corners
    x_coords = np.round(np.linspace(-2, 2, int(4 / grid_resolution) + 1), 2)
    y_coords = np.round(np.linspace(-2, 2, int(4 / grid_resolution) + 1), 2)

    # Construct grid
    grid = [(x, y) for x in x_coords for y in y_coords]

    # print("X coords:", x_coords)
    # print("Y coords:", y_coords)
    # print("Grid size:", len(grid))

    return grid, x_coords, y_coords

def is_point_in_object(point, obj):
    px, py = point
    ox, oy = obj['position'][:2]
    sx, sy = obj['size'][:2] / 2.
    # print(f"Point: {point}, Object: {obj['name']}, Position: {obj['position']}, Size: {obj['size']}")
    # print(f"Object bounds: {ox - sx} <= {px} <= {ox + sx } and {oy - sy} <= {py} <= {oy + sy}")
    return ox - sx <= px <= ox + sx and oy - sy <= py <= oy + sy

def get_neighbors_of_object(obj, grid, grid_resolution:int=0.1):
    neighbors = []
    ox, oy = obj['position'][:2]
    sx, sy = obj['size'][:2]
    min_x, max_x = ox - sx / 2 - grid_resolution, ox + sx / 2 + grid_resolution
    min_y, max_y = oy - sy / 2 - grid_resolution, oy + sy / 2 + grid_resolution
    
    for x, y in grid:
        if min_x <= x <= max_x and min_y <= y <= max_y and not is_point_in_object((x, y), obj):
            neighbors.append((x, y))
    
    return neighbors


def construct_graph(grid, objects, grid_resolution, tolerance=1e-6):
    G = nx.Graph()
    
    for node in grid:
        if not any(is_point_in_object(node, obj) for obj in objects):  # Remove isolated nodes
            G.add_node(node)
    
    for (x1, y1) in G.nodes:  # Only iterate over valid nodes
        for (x2, y2) in G.nodes:
            if (x1, y1) != (x2, y2):  # Avoid self-connections
                
                # Compute Euclidean distance
                dist = np.linalg.norm([x2 - x1, y2 - y1])
                
                # Use `np.isclose()` to avoid floating point errors
                if np.isclose(dist, grid_resolution, atol=tolerance):
                    G.add_edge((x1, y1), (x2, y2), weight=dist)
    
    

    # Remove isolated nodes (nodes with no edges)
    isolated_nodes = [node for node in G.nodes if G.degree(node) == 0]
    G.remove_nodes_from(isolated_nodes)

    # print(f"Removed {len(isolated_nodes)} isolated nodes.")
    # print("Final graph connected:", nx.is_connected(G) if len(G.nodes) > 0 else False)
    # print("Connected components:", nx.number_connected_components(G))

    # print("Graph connected:", nx.is_connected(G))
    return G



def compute_path_density(graph, obj, grid, grid_resolution):
    neighbors = get_neighbors_of_object(obj, grid, grid_resolution)
    density = {node: 0 for node in graph.nodes}
    total_paths = 0
    
    for start in neighbors:
        if start in graph:
            lengths = nx.single_source_shortest_path_length(graph, start)
            for end, length in lengths.items():
                # print(f"Start: {start}, End: {end}, Length: {length}")
                if start != end:
                    total_paths += 1
                    path = nx.shortest_path(graph, start, end)
                    for node in path:
                        density[node] += 1
    
    density = {node: (value / len(graph.nodes)) if len(graph.nodes) > 0 else 0 for node, value in density.items()}
    return density

def find_bottlenecks(density, n=None):
    if not density:
        return []
    
    # Sort nodes in descending order based on density values
    sorted_bottlenecks = sorted(density.items(), key=lambda x: x[1], reverse=True)
    
    # Select the top `n` bottlenecks if `n` is specified
    return [node for node, _ in sorted_bottlenecks[:n]] if n else [node for node, _ in sorted_bottlenecks]


def propose_bottlenecks(config, obj_name, grid_resolution: float =.1, n:int=10):  
 
    objects = get_collidable_objects(config)

    grid, x_coords, y_coords = define_grid(grid_resolution=grid_resolution)

    graph = construct_graph(grid, objects, grid_resolution=grid_resolution)
    
    # # Example usage
    # visualize_grid_graph(graph, grid_resolution=grid_resolution)  # Make sure G is the graph from your construction

    obj = next(obj for obj in objects if obj['name'] == obj_name)  # Get object by name
    #neighbors = get_neighbors_of_object(obj, grid, grid_resolution=grid_resolution*3)
    
    # print(f"Graph has {len(graph.nodes)} nodes and {len(graph.edges)} edges.")
    # print(f"Neighbors of object {obj['name']}: {len(neighbors)} nodes")
    
    density = compute_path_density(graph, obj, grid, grid_resolution=grid_resolution)
    bottlenecks = find_bottlenecks(density, n)
    
    # print("Bottlenecks:", bottlenecks)
    return bottlenecks

# if __name__ == "__main__":
#     file_path = "SeqMan-main/manipulation-puzzles-main/puzzles/p4-four-blocks.g"
#     config = ry.Config()
#     config.addFile(file_path)
#     obj_name = "obj2"  # Specify object name here
#     points = propose_bottlenecks(config, obj_name, 0.2, 10)
    
#     for i, p in enumerate(points):
#         config.addFrame(f"bottleneck{i}", "world", "shape: marker, size: [0.1]")
#         config.frame(f"bottleneck{i}").setPosition([*p, 0.2])
#     config.view(True)