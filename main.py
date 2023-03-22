import mygraph
graph = mygraph.WeightedGraph()
graph.read_from_file("input.txt", False)
graph.print()
distances = {0:9,
    1: 7,
    2: 6,
    3: 5,
    4: 7,
    5: 3,
    6: 2,
    7: 0}
path = graph.a_star_search(1, 7, distances)
print(path)