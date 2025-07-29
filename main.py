from instruction_executor import InstructionExecutor

SP = 14  # Stack Pointer (Ponteiro de Pilha)
PC = 15  # Program Counter (Contador de Programa)

class RISCSimulator:
    """
    Simulador para um processador RISC de 16 bits.
    """

    def __init__(self):
        """
        Inicializa todos os componentes do processador.
        """
        # 16 registradores de 16 bits, todos iniciam em 0 
        self.registers = [0] * 16

        # O ponteiro de pilha (SP) inicia no endereço 0x8000
        self.registers[SP] = 0x8000

        # Flags de estado, ambas iniciam em 0
        self.flags = {'Z': 0, 'C': 0}
        
        # A memória é um dicionário para simular um espaço de endereçamento.
        self.memory = {}

        # Conjunto para rastrear posições de memória de dados acessadas, para o relatório final
        self.accessed_data_memory = set()
        
        # Constantes de registradores especiais
        self.SP = SP
        self.PC = PC
        
        # Inicializa o executor de instruções
        self.instruction_executor = InstructionExecutor(self)

    def load_program_from_hex_string(self, hex_code_string):
        """
        Carrega um programa na memória a partir de uma string multilinhas.

        O formato esperado é '<endereço>:<conteúdo>' em hexadecimal.
        Linhas vazias ou que não seguem o formato são ignoradas.
        """
        print("--- Carregando Programa na Memória ---")
        lines = hex_code_string.strip().split('\n')
        for line in lines:
            if ':' in line:
                try:
                    addr_str, content_str = line.split(':')
                    addr = int(addr_str, 16)
                    content = int(content_str, 16)
                    self.memory[addr] = content
                    print(f"Endereço {addr:04X}h: {content:04X}h")
                except ValueError:
                    print(f"AVISO: Linha ignorada por erro de formato: '{line}'")
        print("--- Carga Finalizada ---\n")

    def print_final_state(self):
        """
        Exibe o estado final do simulador, conforme especificado no trabalho.
        """
        print("\n--- Fim da Execução ---")
        print("Estado final do simulador:")

        # 1. Exibir Registradores 
        print("\n[ Registradores ]")
        for i in range(14): # R0 a R13
            print(f"R{i}: 0x{self.registers[i]:04X}")
        print(f"SP: 0x{self.registers[SP]:04X}") # R14
        print(f"PC: 0x{self.registers[PC]:04X}") # R15
        
        # 2. Exibir Flags [cite: 35]
        print("\n[ Flags ]")
        print(f"Z (Zero): {self.flags['Z']}")
        print(f"C (Carry): {self.flags['C']}")

        # 3. Exibir Memória de Dados Acessada 
        if self.accessed_data_memory:
            print("\n[ Memória de Dados Acessada ]")
            # Ordena os endereços para uma exibição consistente
            for addr in sorted(list(self.accessed_data_memory)):
                print(f"0x{addr:04X}: 0x{self.memory.get(addr, 0):04X}")
            
        # 4. Exibir Pilha (se usada) 
        if self.registers[SP] != 0x8000:
            print("\n[ Pilha ]")
            # A pilha é descendente, então exibimos do SP até a base 0x8000
            current_sp = self.registers[SP]
            while current_sp < 0x8000:
                 print(f"0x{current_sp:04X}: 0x{self.memory.get(current_sp, 0):04X}")
                 current_sp += 1 # A pilha tem alinhamento de 16 bits (2 bytes), mas endereçamento é por palavra
        else:
            print("\n[ Pilha ]")
            print("A pilha não foi utilizada (SP permaneceu em 0x8000).")


    def sign_extend(self, value, bits):
        """
        Estende o sinal de um número de 'bits' para o tamanho de um inteiro do Python.
        Necessário para tratar imediatos com sinal (e.g., em saltos).
        """
        sign_bit = 1 << (bits - 1)
        # Se o bit de sinal estiver ligado, preenche os bits superiores com 1
        if (value & sign_bit) != 0:
            return value - (1 << bits)
        return value

    def run(self):
        """
        Inicia o ciclo de busca, decodificação e execução das instruções.
        O ciclo continua até encontrar a instrução HALT (0xFFFF).
        """
        running = True
        print("--- Iniciando Execução do Simulador ---\n")
        
        while running:
            # 1. BUSCA (Fetch)
            pc_address = self.registers[PC]
            instruction = self.memory.get(pc_address, 0)

            print(f"Executando em PC=0x{pc_address:04X} -> Instrução=0x{instruction:04X}")

            # 2. INCREMENTO DO PC
            self.registers[PC] += 1
            
            # 3. DECODIFICAÇÃO E EXECUÇÃO (Decode & Execute)
            running = self.instruction_executor.execute_instruction(instruction)

        self.print_final_state()


if __name__ == "__main__":
    
    program_code = """
    0000:0x410A
    0001:0x420A
    0002:0x7312
    0003:0x1402
    0004:0x4464
    0005:0x45C8
    0006:0xFFFF
    """

    # Cria uma instância do simulador
    simulator = RISCSimulator()

    # Carrega o programa de exemplo
    simulator.load_program_from_hex_string(program_code)
    
    # Executa o simulador
    simulator.run()