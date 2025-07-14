# Simulação e Controle de Robô SCARA para Tarefa Pick-and-Place

Este projeto apresenta a modelagem, simulação e controle de um robô manipulador do tipo SCARA (Selective Compliance Assembly Robot Arm) com 3 graus de liberdade (R-R-P). O objetivo é executar uma trajetória de "pegar e colocar" (pick-and-place), incluindo a análise de singularidades e o controle dinâmico por meio de um controlador PID com ganhos sintonizados pelo método do Torque Calculado.

## 👥 Participantes

| Nome                | Número USP |
| ------------------- | ---------- |
| Amanda Tanaka       | 12550153   |
| Felipe Mendes       | 12752523   |
| Guilherme Rebecchi  | 12550107   |
| Eduardo Fares       | 12686036   |
| Juan Thomas         | 12685550   |

## 🚀 Como Executar

1.  **Clone o repositório:**
    ```bash
    git clone https://github.com/stitchgui/Controle-de-sistemas-roboticos.git
    cd Controle-de-sistemas-roboticos
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
    python trabalho_final.py
    ```

4.  **Verifique os resultados:**
    -   Os gráficos de análise serão exibidos na tela ao final da execução.
    -   Um arquivo chamado `trajetoria_robo_final.gif` será salvo no diretório do projeto, contendo a animação do robô.

Ao executar o código, você observará:

-   **Saída no terminal:** Informações sobre o modelo do robô, o cálculo dos ganhos PID e avisos de singularidade, caso detectados.
-   **Janelas de Gráficos:** Para cada uma das 3 juntas, serão mostrados três gráficos:
    1.  **Seguimento da Trajetória:** Comparação entre a posição desejada (vermelho tracejado) e a posição real simulada (azul contínuo). Pontos singulares são marcados em vermelho.
    2.  **Evolução do Erro:** O erro de posição ao longo do tempo.
    3.  **Sinal de Controle:** O torque (Juntas 1 e 2) ou força (Junta 3) aplicado pelo controlador.
-   **Gráfico do Determinante do Jacobiano:** Mostra a evolução do módulo do determinante do Jacobiano ao longo da trajetória, destacando os pontos de singularidade.
-   **Arquivo GIF:** Uma animação visual do movimento do robô.
