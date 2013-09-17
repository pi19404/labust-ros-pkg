#!/usr/bin/env python
import os
import argparse

def loadBundles():
    script_path = os.path.dirname(os.path.realpath(__file__))
    try:
        bunfile = open(script_path + "/bundle_list.txt",'r')
        bundle_dict = dict();
        for line in bunfile:
            list = line.strip("\n").split(" ")
            print list
            if not list == ['']:
                bundle_dict[list[0]] = list[1:]
        
        bunfile.close()
        return bundle_dict
    except IOError:
        pass
    
    return {}   

def addBundle(list):
    script_path = os.path.dirname(os.path.realpath(__file__))
    bunfile = open(script_path + "/bundle_list.txt",'a')
    bunfile.write(" ".join(list) + "\n")
    bunfile.close()
    
def getOpts():     
    parser = argparse.ArgumentParser(description="The scripts masks"
     "packages that are not from the desired bundle by adding the" 
     " CATKIN_IGNORE file.")
    parser.add_argument("-b", "--bundle", dest="bundle",
                  help="select bundle", metavar="BUNDLE")
    parser.add_argument("-l", "--list", 
                      action="store_true", dest="list", default=False,  
                      help="list available bundles")
    parser.add_argument("-a", "--add", dest="addlist",
                  help="add a new bundle", nargs="+", metavar="pkg")
    parser.add_argument("-r", "--remove", dest="rmbundle",
                  help="remove a bundle", metavar="BUNDLE")
    return parser.parse_args()

def selectBundle(pkglist):
    script_path = os.path.dirname(os.path.realpath(__file__))
    for folder, subfolders, files in os.walk(script_path+"/.."):
        for file in files:
             if file == "package.xml":
                 pkgname = os.path.split(folder)[1];
                 ignorefile = "/".join([folder, "CATKIN_IGNORE"])
                 if pkgname in pkglist:
                     if os.path.exists(ignorefile): 
                         os.remove(ignorefile)
                         print "Remove CATKIN_IGNORE from {}".format(pkgname)
                 else:
                     if not os.path.exists(ignorefile):
                         open(ignorefile, 'w').close()
                         print "Add CATKIN_IGNORE to {}".format(pkgname)
                    
def writeBundleFile(bundle_dict):
    script_path = os.path.dirname(os.path.realpath(__file__))
    bunfile = open(script_path + "/bundle_list.txt",'w')
    for key in bundle_dict.keys():
        bunfile.write(" ".join([str(key), " ".join(bundle_dict[key])])+ "\n")
    bunfile.truncate()
    bunfile.close()                 
                 

if __name__=="__main__":   
    args = getOpts()
    bundle_dict = loadBundles()
    
    if args.addlist:
        if args.addlist[0] in bundle_dict.keys():
            print "Replace existing bundle [{}] using {} packages.".format(
                                        args.addlist[0], args.addlist[1:])
            bundle_dict[args.addlist[0]] = args.addlist[1:]
            writeBundleFile(bundle_dict) 
        else:
            print "Adding new bundle [{}] using {} packages.".format(
                                        args.addlist[0], args.addlist[1:])
            addBundle(args.addlist)
        
    if args.list:
        for key in bundle_dict.keys():
            print "{}: {}".format(key,str(bundle_dict[key]).strip("[]"))
           
    if args.bundle in bundle_dict.keys():
        print "Select bundle {}.".format(args.bundle)
        selectBundle(bundle_dict[args.bundle])
    elif args.bundle:
        print "Unknown bundle. None selected."
    
    if args.rmbundle in bundle_dict.keys():
        print "Remove bundle {}.".format(args.rmbundle)
        del bundle_dict[args.rmbundle]
        writeBundleFile(bundle_dict)
    elif args.rmbundle:
        print "Unknown bundle. None selected."
    
    
    


