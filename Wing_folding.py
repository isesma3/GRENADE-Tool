# Code to determine the number of wing folds required

def num_wing_folds(b, D_shell):
    if b < D_shell:
        num_fold = 0
        fold_loc = 0
    else:
        excess_wingspan = [b - 1.03*D_shell]  # Initialize the list with a value. D_shell was increased by 3 percent to give the folded wing wiggle room
        num_fold = 0
        i = 0
        while excess_wingspan[i] > 0:
            excess_wingspan.append(excess_wingspan[i]/2 - D_shell)  # Append new values to the list
            num_fold += 1
            i += 1
        fold_loc = b - excess_wingspan[i-1]
    return (2*num_fold, fold_loc)

# [x,y] = num_wing_folds(4.75,4.5)
# print(x)
# print(y)