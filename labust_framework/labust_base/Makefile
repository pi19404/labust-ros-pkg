all: pull checkout labust-base

GITREPO = https://github.com/labust/labust-base.git
BUILDDIR = .
CHECKOUTDIR = $(BUILDDIR)/labust-base

checkout: $(BUILDDIR)/checkout
		  git clone $(GITREPO) $(CHECKOUTDIR)
		  touch $(BUILDDIR)/checkout

pull: checkout
	  (cd $(CHECKOUTDIR) ; git fetch; git pull)

labust-base: pull checkout
	  (cd $(CHECKOUTDIR) ; ./build.sh -j4)

eclipse-project: pull checkout
	  mkdir -p $(CHECKOUTDIR)/build/labust-base
	  (cd $(CHECKOUTDIR)/build/labust-base ; cmake -G"Eclipse CDT4 - Unix Makefiles" ../../src)

clean:
	        -rm -rf $(BUILDDIR)/labust-base/build
wipe: clean
	        -rm -rf labust-base
	        -rm -rf $(BUILDDIR)/checkout
