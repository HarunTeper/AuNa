sudo apt-get install ansible

ansible-playbook f110_host_setup.yml --ask-become-pass

newgrp docker
docker run hello-world