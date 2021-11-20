# take a yaml instance with map and agents and convert to moving ai instance.
import yaml

def yaml_to_mvai(yaml_fname, map_fname, agents_fname):
    data = yaml.load(open(yaml_fname), yaml.Loader)

    map = data["map"]
    mf = open(map_fname, "w")
    asci = [['.' for _ in range(map['dimensions'][1])] for _ in range(map['dimensions'][0])]
    for ob in map["obstacles"]:
        asci[ob[1]][ob[0]] = '@'
    mf.write("the first char has to be t\n")
    mf.write("height {}\n".format(map["dimensions"][0]))
    mf.write("width {}\n".format(map["dimensions"][1]))
    mf.write("map\n")
    for row in asci:
        for item in row:
            mf.write(item)
        mf.write("\n")
    
    
    agents = data["agents"]
    agf = open(agents_fname, "w")
    agf.write("version 1\n")
    for i in agents:
        agf.write("{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}\n".format(0, map_fname, map["dimensions"][0], map["dimensions"][1], i["start"][0], i['start'][1], i['goal'][0], i['goal'][1], 1729))

if __name__ == "__main__":
    yamlfname = "./parsetest.yaml"
    yaml_to_mvai(yamlfname, "parsetest.map", "parsetest.agents")