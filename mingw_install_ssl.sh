wget -c https://www.openssl.org/source/openssl-1.0.2p.tar.gz
tar -xzvf openssl-1.0.2p.tar.gz
cd openssl-1.0.2p/
./Configure --prefix=/usr/local/openssl no-idea no-mdc2 no-rc5 shared mingw
make depend
make
make test
sudo make install 
