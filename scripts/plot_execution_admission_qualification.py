"""绘制已核验限定验证的首个预声明配对与全部种子，不按结果选例。"""
import argparse
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from admission_execution_study import digest


def plot(root):
    v=json.loads((root/'verified_analysis_v2.json').read_text())
    q=json.loads((root/'qualification_decision.json').read_text())
    if not v['verified'] or v['state']!='COMPLETED' or q['verification_sha256']!=digest(root/'verified_analysis_v2.json'):
        raise ValueError('必须使用完整已核验组')
    m=json.loads((root/'manifest.json').read_text());rows=json.loads((root/'results.json').read_text())
    status=json.loads((root/'status.json').read_text());lookup={(r['trial_id'],r['condition']):r for r in rows}
    colors={'GEOMETRY':'#c66a20','NOMINAL_DYNAMIC':'#2166a5','HOLD':'#777777'}
    labels={'GEOMETRY':'Geometry admission','NOMINAL_DYNAMIC':'Dynamic admission','HOLD':'Safe hold'}
    plt.rcParams.update({'font.size':11,'axes.spines.top':False,'axes.spines.right':False,'pdf.fonttype':42})
    fig,axes=plt.subplots(1,2,figsize=(9.1,4.8),gridspec_kw={'width_ratios':[1,1.25]})
    trial=m['trials'][0]; ax=axes[0]
    for condition in m['conditions']:
        row=lookup[trial['trial_id'],condition]
        a=next(a for a in status['attempts'] if a['state']=='VALID' and a['label'].startswith(trial['trial_id']+'_'+condition+'_attempt'))
        records=[json.loads(line) for line in (Path(a['summary']).parent/'trajectory.jsonl').read_text().splitlines()]
        xy=np.array([r['drones']['3']['pos'][:2] for r in records])
        ax.plot(*xy.T,color=colors[condition],lw=1.8,label=labels[condition])
        if condition!='HOLD':
            decision=row['admission']['decision_state']['3'];start=np.array(decision['position'][:2]); route=np.array([start.tolist()]+[p[:2] for p in row['admission']['route']])
            ax.plot(*route.T,color=colors[condition],lw=.9,ls='--',alpha=.5)
            ax.scatter(*route[1,:],color=colors[condition],marker='s',s=25,zorder=4)
    geom=lookup[trial['trial_id'],'GEOMETRY'];state=geom['admission']['decision_state']; p=np.array(state['3']['position'][:2]);vel=np.array(state['3']['velocity'][:2])
    ax.annotate('',xy=p+1.2*vel,xytext=p,arrowprops={'arrowstyle':'->','lw':1.5,'color':'#333333'})
    ax.text(p[0]-2.7,p[1]+.4,'Velocity at failure',fontsize=9,bbox={'facecolor':'white','edgecolor':'none','alpha':.95,'pad':1})
    failed=np.array(state['2']['position'][:2]);ax.scatter(*failed,marker='x',s=70,color='black',lw=1.8,zorder=5)
    ax.text(failed[0]-.7,failed[1]+.5,'Failed UAV',fontsize=9)
    goal=m['geometries'][trial['geometry_id']]['critical_goal'];ax.scatter(*goal,marker='*',s=110,color='#277340',zorder=5);ax.text(goal[0]+.25,goal[1]-.15,'Goal',fontsize=9)
    ax.set(xlabel='$x$ (m)',ylabel='$y$ (m)',title='(a) First frozen pair: seed 43001');ax.set_aspect('equal');ax.grid(alpha=.15)
    ax=axes[1];names=list(m['geometries']);offsets={'GEOMETRY':-.2,'NOMINAL_DYNAMIC':0,'HOLD':.2};markers={'GEOMETRY':'o','NOMINAL_DYNAMIC':'o','HOLD':'^'}
    for i,name in enumerate(names):
        trials=[t for t in m['trials'] if t['geometry_id']==name]
        for rep,t in enumerate(trials):
            jitter=(rep-.5)*.07
            xx=[i+offsets[c]+jitter for c in m['conditions']];yy=[lookup[t['trial_id'],c]['min_distance_m'] for c in m['conditions']]
            ax.plot(xx,yy,color='#aaaaaa',lw=.7,alpha=.7,zorder=1)
            for c,x,y in zip(m['conditions'],xx,yy):ax.scatter(x,y,s=35,marker=markers[c],color=colors[c],edgecolor='white',linewidth=.4,zorder=3)
    ax.axhline(2.4,color='black',ls='--',lw=1);ax.text(.2,2.65,'Task separation: 2.4 m',fontsize=9,bbox={'facecolor':'white','edgecolor':'none','alpha':.9,'pad':1})
    ax.set_xticks(range(6),['Quarter\nturn','Half\nturn','Reflect','Translate','Aligned','Occupied'],fontsize=9)
    ax.set(ylabel='Episode minimum center separation (m)',title='(b) All 12 frozen pairs',ylim=(1.4,5.6),xlim=(-.5,5.5));ax.grid(axis='y',alpha=.15)
    handles,legends=axes[0].get_legend_handles_labels();fig.legend(handles,legends,loc='upper center',ncol=3,frameon=False,bbox_to_anchor=(.5,1.0),fontsize=10)
    fig.tight_layout(rect=(0,0,1,.92))
    outputs=[]
    for suffix in ('pdf','png'):
        path=root/('execution_admission_qualification_v1.'+suffix);fig.savefig(path,dpi=220,bbox_inches='tight');outputs.append(path)
    plt.close(fig)
    (root/'figure_receipt.json').write_text(json.dumps({'verification_sha256':q['verification_sha256'],'representative_trial':trial['trial_id'],'selection_rule':'first prespecified trial, not selected by outcome','outputs':{str(p):digest(p) for p in outputs}},indent=2))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('root',type=Path);args=p.parse_args();plot(args.root)
