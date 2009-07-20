dist-deb: dist
	tar xzf libbarrett-0.1.tar.gz
	cp -r pkg/debian libbarrett-0.1
	cd libbarrett-0.1 && debuild -us -uc
	rm -rf libbarrett-0.1
	
dist-deb-clean:
	rm -f libbarrett0_0.1_i386.deb
	rm -f libbarrett-dev_0.1_i386.deb

	rm -f libbarrett_0.1.tar.gz
	rm -f libbarrett_0.1_i386.build
	rm -f libbarrett_0.1_i386.changes
	rm -f libbarrett_0.1.dsc
