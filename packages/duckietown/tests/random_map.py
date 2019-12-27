from duckietown_utils.map_utils import RandomMapGenerator

if __name__ == "__main__":
    size = (4, 4)
    rmg = RandomMapGenerator(size)
    the_map = rmg.generate()
    print(the_map)
