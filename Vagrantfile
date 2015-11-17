# -*- mode: ruby -*-
# vi: set ft=ruby :

Vagrant.configure(2) do |config|
  config.vm.box = "ubuntu/trusty64"

   config.vm.provision "shell", inline: <<-SHELL
     sudo apt-get update -y
     sudo apt-get install -y build-essential
     sudo apt-get install -y git
     sudo apt-get install -y libjsoncpp-dev libjsoncpp0
   SHELL
end
