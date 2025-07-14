# Simulação e Controle de Robô SCARA para Tarefa Pick-and-Place

Este projeto apresenta a modelagem, simulação e controle de um robô manipulador do tipo SCARA (Selective Compliance Assembly Robot Arm) com 3 graus de liberdade (R-R-P). O objetivo é executar uma trajetória de "pegar e colocar" (pick-and-place), incluindo a análise de singularidades e o controle dinâmico por meio de um controlador PID com ganhos sintonizados pelo método do Torque Calculado.

## 👥 Participantes

| Nome                | Número USP |
| ------------------- | ---------- |
| Amanda Tanaka       | 12550153   |
| Felipe Mendes       | 12752523   |
| Guilherme Rebecchi  | 12550107   |

## 📝 Descrição do Projeto

O código desenvolve uma simulação completa de um robô SCARA, desde sua definição cinemática e dinâmica até a execução de uma tarefa complexa. As principais etapas do projeto são:

1.  **Modelagem do Robô:** Definição do robô utilizando a convenção de Denavit-Hartenberg (DH) e atribuição de parâmetros dinâmicos (massa dos elos e centro de massa) utilizando a biblioteca `Robotics Toolbox for Python`.
2.  **Cinemática Inversa:** Implementação de uma função analítica para resolver a cinemática inversa, permitindo calcular as configurações de junta necessárias para alcançar pontos específicos no espaço cartesiano.
3.  **Planejamento de Trajetória:** Geração de uma trajetória suave no espaço das juntas usando a função `jtraj` para conectar os pontos da tarefa (ir até o objeto, pegar, mover para o local de destino, colocar).
4.  **Controle Dinâmico PID:** Simulação de um controlador PID que calcula os torques e forças necessários para que o robô siga a trajetória desejada. Os ganhos do controlador (`Kp`, `Ki`, `Kd`) são calculados sistematicamente com base no modelo dinâmico do robô.
5.  **Análise de Singularidade:** Durante a simulação, o Jacobiano do robô é constantemente monitorado para detectar configurações singulares, onde o robô perde graus de liberdade e sua controlabilidade é afetada.

## ✨ Funcionalidades Principais

-   **Modelo DH Parametrizado:** Fácil alteração dos comprimentos dos elos (`L1`, `L2`).
-   **Trajetória Pick-and-Place Completa:** Movimentos verticais e horizontais para simular uma tarefa realista.
-   **Controlador PID com Compensação de Gravidade e Coriolis:** O controle é robusto, pois leva em conta a dinâmica completa do robô.
-   **Detecção e Relatório de Singularidades:** O sistema avisa em tempo real sempre que a trajetória passa por uma singularidade, analisando o determinante e o posto do Jacobiano.
-   **Visualização Gráfica:** Geração de gráficos detalhados para cada junta, mostrando a posição desejada vs. a obtida, o erro ao longo do tempo e o sinal de controle (torque/força).
-   **Animação da Trajetória:** Criação de um arquivo GIF (`trajetoria_robo_final.gif`) que mostra o robô executando a tarefa, permitindo uma análise visual do comportamento.

## 🛠️ Tecnologias Utilizadas

-   **Python 3**
-   **Robotics Toolbox for Python (`roboticstoolbox-python`):** Para modelagem, planejamento de trajetória e simulação.
-   **NumPy:** Para cálculos numéricos e manipulação de arrays.
-   **SpatialMath (`spatialmath-python`):** Para operações de geometria e transformações no espaço 3D.
-   **Matplotlib:** Para a geração de todos os gráficos de análise.

## 🚀 Como Executar

1.  **Clone o repositório:**
    ```bash
    git clone <URL-DO-SEU-REPOSITORIO>
    cd <NOME-DO-REPOSITORIO>
    ```

2.  **Instale as dependências:**
    É recomendado criar um ambiente virtual.
    ```bash
    python -m venv venv
    source venv/bin/activate  # No Windows: venv\Scripts\activate
    ```
    Instale as bibliotecas necessárias:
    ```bash
    pip install roboticstoolbox-python numpy matplotlib
    ```

3.  **Execute o script principal:**
    ```bash
    python nome_do_seu_arquivo.py
    ```

4.  **Verifique os resultados:**
    -   Os gráficos de análise serão exibidos na tela ao final da execução.
    -   Um arquivo chamado `trajetoria_robo_final.gif` será salvo no diretório do projeto, contendo a animação do robô.

## 📊 Resultados Esperados

Ao executar o código, você observará:

-   **Saída no terminal:** Informações sobre o modelo do robô, o cálculo dos ganhos PID e avisos de singularidade, caso detectados.
-   **Janelas de Gráficos:** Para cada uma das 3 juntas, serão mostrados três gráficos:
    1.  **Seguimento da Trajetória:** Comparação entre a posição desejada (vermelho tracejado) e a posição real simulada (azul contínuo). Pontos singulares são marcados em vermelho.
    2.  **Evolução do Erro:** O erro de posição ao longo do tempo.
    3.  **Sinal de Controle:** O torque (Juntas 1 e 2) ou força (Junta 3) aplicado pelo controlador.
-   **Gráfico do Determinante do Jacobiano:** Mostra a evolução do módulo do determinante do Jacobiano ao longo da trajetória, destacando os pontos de singularidade.
-   **Arquivo GIF:** Uma animação visual do movimento do robô.

![Exemplo da Animação do Robô](trajetoria_robo_final.gif)
*(Para que esta imagem funcione, o arquivo GIF deve estar no mesmo diretório que o README.md)*
