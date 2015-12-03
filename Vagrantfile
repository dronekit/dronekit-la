# -*- mode: ruby -*-
# vi: set ft=ruby :

Vagrant.configure(2) do |config|
  config.vm.box = "ubuntu/vivid64"

   config.vm.provision "devenv", type: "shell", inline: <<-SHELL
     sudo apt-get update -y
     sudo apt-get install -y git
     sudo apt-get install -y build-essential
     sudo apt-get install -y libjsoncpp-dev libjsoncpp0
   SHELL

   config.vm.provision "sphinx", type: "shell", inline: <<-SHELL
        echo "[SPHINX-DOCS]: Installing Python Devel"
        apt-get -y install python-dev
        echo "[SPHINX-DOCS]: Installing pip ..."
        apt-get -y install python-pip
        easy_install -U pip     
        echo "[SPHINX-DOCS]: Installing Sphinx ... "
        pip install sphinx
        cd /vagrant
        echo "[SPHINX-DOCS]: Installing SPHINX-DOCS theme ... "
        pip install sphinx-3dr-theme
        echo "[SPHINX-DOCS]: Installing Breath Extension (Doxygen->Spinx Bridge) ... "
        pip install breathe
        echo "[SPHINX-DOCS]: Installing DOxygen ..."
        apt-get -y install doxygen  
        echo "[SPHINX-DOCS]: Run DOxygen ..."
        doxygen doxygen/doxygen.conf
        echo "[SPHINX-DOCS]: Building docs "
        cd docs/
        make html
    SHELL
end
