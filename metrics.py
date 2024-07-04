
import pickle as pkl

def compute_random_metrics():
    
    path = "scenarios/random_results.pkl"
    with open(path, "rb") as f:
        r = pkl.loads(f.read())

    results = {}
    for ri in r:
        for k, v in ri.items():
            if k in results.keys():                
                results[k] = dict_append_value(results[k], 'gp_length', v['gp_length'])
                results[k] = dict_append_value(results[k], 'ap_length', v['ap_length'])
                results[k] = dict_append_value(results[k], 'tt', v['tt'])
            else:
                results[k] = {'gp_length': [v['gp_length']], 'ap_length': [v['ap_length']], 'tt': [v['tt']]}

    for k, v in results.items():
        print("**************")
        print(k)
        for metric, value in v.items():
            print(metric, "mean:", sum(value)/len(value))
        
        tls = [gl+al for gl, al in zip(list(v['gp_length']), list(v['ap_length']))]
        print("total path length  mean:", sum(tls)/len(tls))
        print(len(tls), tls[:5])
        print()


def dict_append_value(d, key, val):

    try:
        d[key].append(val)
    except:
        d[key] = [val]

    return d




# gp_length mean: 27.942751130696156 
# ap_length mean: 36.08415522719455  
# tt mean: 0.5368906673119993        
# total path length  mean: 64.02690635789078

if __name__ == "__main__":

    compute_random_metrics()