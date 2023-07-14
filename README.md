<h1 align="center"> Localização Robótica baseada no Perfect Match </h1>

Licença: Universidade Federal da Bahia(UFBA);
Data da última versão do projeto: junho.

# Resumo do projeto
	Projeto finalizado. Ele consiste no desenvolvimento de ferramentas computacionais para navegação e localização robótica, focado principalmente no Perfect Matche no AMCL. Nele foram feitos ambientes de simulação utilizando o Husky para testar algumas hipóteses e tentar replicar resultados de outros estudos.

## 🔨 Conteúdo do projeto

- `Modelo do departamento`: Foi realizada a modelagem 3D do Departamento de Engenharia Elétrica e de Computação utilizando o software SketchUp, após isso foi utilizado o blender para otimizar a quantidade de polígonos no modelo, e depois exportado como .dae, posteriormente, ele foi inserido e simulado no software Gazebo. O modelo está localizado em "catkin_ic/models/corredor/meshes/dpto_eletrica.dae". Já o arquivo .world relativo à ele é "catkin_ic/worlds/departamento.world".

- `Plugin do gazebo`: Seguindo a tese de mestrado (Brennan, 2019), presente no repositório, foi desenvolvido um plugin para o Gazebo para a tentativa de replicação dos dados obtidos pelo estudo. O plugin desenvolvido é responsável por publicar sequestros periódicos na forma de "teletransportes" das coordenadas do robô simulado, onde os parâmetros do sequestro e o timer são facilmente configuráveis, além disso, é facil aumentar o número de sequestros realizados, apenas colocando outras funções que serão chamadas por timers, que também pode ser de ativação única. O plugin está localizado em "catkin_ic/scripts/teste.cc", e o plugin usado como referência é o "catkin_ic/scripts/gazeboplugin.cpp", vale notar que o plugin de referência possui diversos erros até mesmo pontuados pela autora do estudo, publicado apenas com o objetivo de ajudar quem estivesse desenvolvendo projetos similares. Esse plugin é um plugin do tipo world, ou seja, deve ser declarado no arquivo .world no qual ele vai ser utilizado. Nesse caso, ele está presente no arquivo "catkin_ic/worlds/corredor_geom.world", um simples ambiente feito para testar esse plugin.

- `Bash`: Alguns scripts em bash foram utilizados para fazer os testes serem mais rápidos. Na pasta "catkin_ic/bash" existem diversos scripts, majoritariamente para a execução de diversos arquivos .launch e até mesmo outros bashes, isso é feito dessa forma por causa da necessidade do ROS de rodar em várias janelas diferentes, então cada roslaunch é feito utilizando um bash diferente, por causa da necessidade da declaração de algumas variáveis específicas em alguns terminais. Além disso, temos o killeverything.sh, responsável por desligar todos os processos relacionados ao gazebo e ao ROS de uma vez.

- `Arquivos .launch`: Foram utilizados diversos arquivos .launch, localizados em "catkin_ic/launch", eles estão bem modularizados por causa da necessidade de fazer vários testes parciais utilizando os scripts bash.


## ✔️ Técnicas e tecnologias utilizadas

- ``ROS``
- ``C++``
- ``XML``
- ``CMAKE``
- ``Bash``
