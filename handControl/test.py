from collections import Counter

A=[1,2,2,2]

counter=Counter(A)

best=counter.most_common(4)

print(best[2])