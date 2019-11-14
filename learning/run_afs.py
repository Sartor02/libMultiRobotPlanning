import exec_with_proc as rp

def run(afs_map, afs_agents, timeout):
    cmd = "timeout {} afs/AnytimeMAPF/driver --map {} --agents {} --export_results /tmp/afs.results --time_limit {}".format(timeout + 2, afs_map, afs_agents, timeout)
    retcode = rp.run_with_current_proc(cmd)
    if retcode != 0:
        print("AFS timeout")
        # AFS dumps the file at the end; if it timed out, that mean AFS died and we have no data.
        return ([2], [timeout])
    f = open("/tmp/afs.results")
    lines = f.readlines()[1:]
    csv = [e.split(',') for e in lines]
    times = [float(e[1]) for e in csv]
    bounds = [float(e[3]) for e in csv]
    # If optimal solution was not found, add max time.
    if (len(bounds) == 0 or bounds[-1] != 1.0):
        times.append(timeout)
        bounds.append(1.0)
    return (bounds, times)