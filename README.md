# Implementação do código da RRT - Doutorado de Luiz Gustavo Diniz de Oliveira Véras

Neste repositório está o código usado durante o meu doutorado. O documento da tese pode ser visto na biblioteca do INPE neste [link](http://mtc-m21c.sid.inpe.br/attachment.cgi/sid.inpe.br/mtc-m21c/2019/08.07.15.41/doc/publicacao.pdf).

O programa foi desenvolvido com Microsoft Visual Studio 2019 (conjunto de ferramentas v142), em linguagem C++ (std:c++14). Portanto o código não foi testado em ferramentas GNU. 

A seguir estão listadas as dependências necessárias o funcionamento do programa. Elas não estão no repositório, portanto é necessário baixa-las e configura-las no projeto onde o código do repositório será incorporado.

* freglut: API para trabalhar com openGL
* pngwriter64: gerador de imagens PNG para salvar as rotas planejadas pela RRT
* libpng64: gerador de imagens PNG para salvar as rotas planejadas pela RRT
* zlib64:  gerador de imagens PNG para salvar as rotas planejadas pela RRT 
* CGAL-5.1: Biblioteca para operações de geometria computacional. Uma das principais utilizadas no programa.
* ANN (neste repositório): Para operações de vizinhos mais próximos. A implementação da kd-tree é utilizada.
* Eingen: Biblioteca para operações de algebra linear
* openGl (vem com o sistema operacional): Biblioteca gráfica
* vcpkg: gerenciador de pacotes C++ no Windows
* PHQuintic: Implementação da curva hodográfica de pitagoras (usado nas simulações com suavização).

Para ajudar, seguem algumas configurações de um projeto do visual studio code para as dependências.

![Diretórios de inclusão](https://user-images.githubusercontent.com/13111432/136489493-7258393b-f1ff-4d58-b788-13f82c5a85e9.png)
![Diretórios de linker](https://user-images.githubusercontent.com/13111432/136489538-c4de6f71-6bfe-49bd-8903-8d6ed56dc2de.png)

## Orientações gerais

O código do programa ainda está bastante desorganizado, então seguem algumas informaões que podem ajudar na execução do mesmo.
* O programa recebe como um dos inputs um arquivo TXT com uma matriz quadrada que representa o ambiente de navegação. Os arquivos podem ser visto no diretório https://github.com/lgveras/rrt_navegacao/tree/main/RRT/RRTSimulations/navEnvs.
* Algumas configurações de paths estão hard coded. Elas são definidas quando uma instância de RRT é criada dentre as opções do bloco swtich-case do arquivo main.cpp. Elas sãos especificadas de acordo com o tipo de RRT e ambiente de navegação, e apontam para diretórios de saída dos resultados da simulação de acordo com a lógica das pastas byIteration e bySeed no diretório https://github.com/lgveras/rrt_navegacao/tree/main/RRT/RRTSimulations.
* Ver os tipos de RRT implementadas no arquivo [inputs.txt](https://github.com/lgveras/rrt_navegacao/blob/main/RRT/inputs.txt).
* O programa gera como saída arquivos .txt com o tempo de execução de cada iteração da RRT, resultado por semente de simulação e visualizações de cada iteração em OpenGL. As imagens dessas visualizações podem ser salvar em PNG. O método saveToFile da classe Graphics.cpp faz isso. Ela é invocada pelos métodos de build de rota nas classes do tipo RRT. Essa configuração também está hard coded.

## Exemplo de argumento

O programa recebe os seguintes argumentos na linha de comando para executar um planejamento.
<caminho para o arquivo do ambiente de navegação> <variante da RRT> <nome do diretório> <n_iterações> <x_init> <y_init> <y_goal> <y_goal> <semente para função aleatória>

"C:/RRTSimulations/navEnvs/binary_grid_40x40_cluttered5Obstacles.txt" 1 "cluttered5Obstacles" 6000 250 250 900 900 5
