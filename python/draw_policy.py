import pylab as mpl
import matplotlib.pyplot as plt; plt.rc('text', usetex=True)
import matplotlib.pyplot as plt
font = {'family' : 'serif',
        'size'   : 8}
plt.rc('font', **font)

import numpy as na
from scipy.special import betainc
import matplotlib.markers

def drawThresholds():
    """ Draw the thresholds in mpl. """
    mpl.figure(figsize=(3.5, 3.5))
    policy = computePolicy()
    #max_idx = na.sqrt(len(policy))
    max_idx = 16

    mpl.xticks([i for i in na.arange(0, max_idx)])
    mpl.yticks([i for i in na.arange(0, max_idx)])
    mpl.xlabel("# of Observed Successes")
    mpl.ylabel("# of Observed Failures")
    mpl.title("Policy for Exploration")

    mpl.gca().spines['right'].set_visible(False)
    mpl.gca().spines['top'].set_visible(False)
    mpl.gca().xaxis.set_ticks_position('bottom')
    mpl.gca().yaxis.set_ticks_position('left')

    accept_X = []
    accept_Y = []

    reject_X = []
    reject_Y = []

    continue_X = []
    continue_Y = []

    for successes in na.arange(0, max_idx):

        for failures in na.arange(0, max_idx):
            result = policy.get((successes, failures))
            if result == "r":
                reject_X.append(successes)
                reject_Y.append(failures)
            elif result == "a":
                accept_X.append(successes)
                accept_Y.append(failures)
            elif result == "t":
                accept_X.append(successes)
                accept_Y.append(failures)
            elif result == "c":
                continue_X.append(successes)
                continue_Y.append(failures)
            else:
                raise ValueError("Unexpected result: " + `result`)
                
    mpl.scatter(accept_X, na.array(accept_Y) - 0.2, marker="s", color="g", s=150)
    mpl.scatter(reject_X, na.array(reject_Y) - 0.2, marker="s", color="r", s=150)

    mpl.scatter(accept_X, na.array(accept_Y) - 0.2, marker="+", color="k", s=10)
    mpl.scatter(reject_X, na.array(reject_Y) - 0.2, marker="_", color="k", s=10)

    for (mx, my) in zip(continue_X, continue_Y):
        mpl.arrow(mx - 0.25, my - 0.25 - 0.2, 0, 0.75, head_width=0.2, length_includes_head=True)
        mpl.arrow(mx - 0.25, my - 0.25 - 0.2, 0.75, 0, head_width=0.2, length_includes_head=True)


    mpl.axis('equal')
    mpl.axis((-0.5, max_idx - 1, -0.5, max_idx - 1)) 
    mpl.savefig("policy.pdf")
    mpl.show()

def printThresholds():
    """ Print the thresholds as ascii art. """

    policy = computePolicy()
    max_idx = na.sqrt(len(policy))
    print "".rjust(2),
    for x in na.arange(0, max_idx):
        print ("%d" % x).rjust(2),
    print
    for successes in na.arange(0, max_idx):
        print ("%d" % successes).rjust(2),
        for failures in na.arange(0, max_idx):
            result = policy.get((successes, failures))
            if (successes + failures <= 10):
                print str(result).rjust(2), 
            else:
                print " ".rjust(2),

            #print successes, failures, mu, "p(s): %.4f" %(1 - probability), 
            #print "p(f): %.4f" % probability
        print


def computePolicy():
    """ Draw the thresholds in matplotlib. """
    max_idx = 20
    target_mu = 0.7
    epsilon = 0.2
    threshold_confidence = 0.7
    accept_confidence = 0.7
    reject_confidence = 0.95

#double algorithmCEPS = 0.2;
#double algorithmCTarget = 0.7;
#double algorithmCAT = 0.7;
#double algorithmCRT = 0.95;



    policy = dict()

    for successes in na.arange(0, max_idx):
        for failures in na.arange(0, max_idx):
            result = compute_policy(successes, failures, target_mu, epsilon, 
                                    threshold_confidence, accept_confidence, reject_confidence)
            policy[(successes, failures)] = result
    return policy

def compute_policy(successes, failures, mu, epsilon, threshold_confidence, accept_confidence, reject_confidence):
    p_in_threshold = (
        betainc(successes + 1, failures + 1, mu + epsilon) - 
        betainc(successes + 1, failures + 1, mu - epsilon))

    p_below = betainc(successes + 1, failures + 1, mu)
    p_above = 1 - p_below
    if p_above > accept_confidence:
        result = "a"
    elif p_in_threshold > threshold_confidence:
        result = "t"
    elif p_below > reject_confidence:
        result = "r"
    else:
        result = "c"
    # result = "c"
    if successes == 0 and failures == 0:
        result = 'c'
    elif successes == 0:
        result = 'r'
    
    # if p_above > accept_confidence:
    #     result = "a"
    # else:
    #     result = "c"

    # if p_in_threshold > threshold_confidence:
    #     result = "t"
    # if p_below > reject_confidence:
    #     result = "r"
    # else:
    #     result = "c"




    return result

def main():
    #printThresholds()
    drawThresholds()

if __name__ == "__main__":
    main()


