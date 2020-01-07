wget -c https://www.openssl.org/source/openssl-1.0.2p.tar.gz
tar -xzvf openssl-1.0.2p.tar.gz
cd openssl-1.0.2p/
./config
make
make test
sudo make install 
