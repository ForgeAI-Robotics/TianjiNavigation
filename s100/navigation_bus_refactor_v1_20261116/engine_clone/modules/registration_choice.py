"""A large correction needs useful paired-scan improvement, not just convergence."""
import math

def retain_validated_prior(prior,candidate,translation,rotation):
 if translation<=.02 and rotation<=math.radians(1.):return False
 if set(prior)!={'front','back'}:return False
 if not all(m['overlap']>.75 and m['rmse']<.065 and m['observable']for m in prior.values()):return False
 # RMSE computed only on inliers can improve by discarding points. Penalize
 # lost support, then require a useful improvement on BOTH observed radars.
 improves=[]
 for sensor in ('front','back'):
  a,b=prior[sensor],candidate[sensor]
  support_ok=b['overlap']>=a['overlap']-.003
  error_gain=a['rmse']-b['rmse']>=max(.003,.05*a['rmse'])
  support_gain=b['overlap']-a['overlap']>=.02 and b['rmse']<=a['rmse']+.001
  improves.append(support_ok and (error_gain or support_gain))
 return not all(improves)
