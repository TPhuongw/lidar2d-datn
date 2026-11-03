import re, sys, design
def parse(path):
    txt=open(path).read(); txt=txt[txt.index("(nets"):]
    nets={}
    for blk in txt.split("(net (code")[1:]:
        name=re.search(r'\(name "([^"]*)"\)',blk).group(1).lstrip("/")
        nets[name]=set(re.findall(r'\(node \(ref "([^"]+)"\) \(pin "([^"]+)"\)',blk))
    return nets
def expected():
    exp={}
    for ref,_,_,_,_,_,_,pm in design.PARTS:
        for pin,net in pm.items():
            if net: exp.setdefault(net,set()).add((ref,pin))
    return exp
if __name__=="__main__":
    nets=parse(sys.argv[1]); exp=expected()
    for i,(net,_,_) in enumerate(design.PWR_FLAGS): exp[net].add((f"#FLG0{i+1}","1"))
    bad=0
    for net,nodes in exp.items():
        if nets.get(net)!=nodes: bad+=1; print("MISMATCH",net,sorted(nodes ^ nets.get(net,set())))
    extra=[n for n in nets if n not in exp]
    single=[n for n,v in nets.items() if len(v)<2 and n in exp]
    print("expected",len(exp),"got",len(nets),"mismatch",bad,"extra",extra,"single-node nets",single)
