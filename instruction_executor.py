class InstructionExecutor:
    """
    Classe responsável pela decodificação e execução das instruções do processador RISC.
    """
    
    def __init__(self, simulator):
        """
        Inicializa o executor com uma referência ao simulador principal.
        
        Args:
            simulator: Instância do RISCSimulator que contém registradores, flags e memória
        """
        self._simulator = simulator
        
        self._instruction_map = {
            0b0000: self._execute_jmp,     # JMP #Im
            0b0100: self._execute_mov,     # MOV Rd, #Im
            0b0111: self._execute_sub,     # SUB Rd, Rm, Rn
            0b0001: self._execute_jcond,   # J<cond> #Im
            0b0101: self._execute_add,     # ADD Rd, Rm, Rn
            0b0110: self._execute_addi,    # ADDI Rd, Rm, #Im
            0b0011: self._execute_str,     # STR Rn, [Rm]
            0b1000: self._execute_subi,    # SUBI Rd, Rm, #Im
            0b1001: self._execute_and,     # AND Rd, Rm, Rn
            0b1010: self._execute_or,      # OR Rd, Rm, Rn
            0b1011: self._execute_shr,     # SHR Rd, Rm, #Im
            0b1100: self._execute_shl,     # SHL Rd, Rm, #Im
            0b1110: self._execute_push,    # PUSH Rn
            0b1111: self._execute_pop,     # POP Rn
            0b0010: self._execute_ldr,     # LDR Rd, [Rm]
            0b1101: self._execute_cmp,     # CMP Rm, Rn
        }
    
    @property
    def registers(self):
        """Acesso aos registradores do simulador."""
        return self._simulator.registers
    
    @property
    def flags(self):
        """Acesso às flags do simulador."""
        return self._simulator.flags
    
    @property
    def memory(self):
        """Acesso à memória do simulador."""
        return self._simulator.memory

    @property
    def accessed_memory(self):
        """Acesso ao rastreio da memória de dados acessada."""
        return self._simulator.accessed_data_memory

    def sign_extend(self, value, bits):
        """Utiliza o método sign_extend do simulador."""
        return self._simulator.sign_extend(value, bits)
    
    def execute_instruction(self, instruction):
        """
        Executa uma instrução baseada no seu opcode.
        
        Args:
            instruction: Instrução de 16 bits a ser executada
            
        Returns:
            bool: True se a execução deve continuar, False se deve parar (HALT)
        """
        # --- INSTRUÇÃO HALT ---
        # A instrução HALT para a execução 
        if instruction == 0xFFFF:
            print("Instrução: HALT")
            return False

        # Decodifica o opcode (4 bits mais significativos) 
        opcode = instruction >> 12
        
        # Busca a função correspondente ao opcode no dicionário
        if opcode in self._instruction_map:
            return self._instruction_map[opcode](instruction)
        else:
            print(f"ERRO: Opcode 0x{opcode:X} desconhecido ou não implementado.")
            return False # Para a simulação em caso de instrução inválida
    
    def _execute_mov(self, instruction):
        """
        MOV Rd, #Im  (Opcode: 0100)
        Carrega um valor imediato de 8 bits em um registrador de destino (Rd).
        """
        rd = (instruction >> 8) & 0xF  # Bits 11-8 para Rd
        imm = instruction & 0xFF        # Bits 7-0 para o imediato
        self.registers[rd] = self.sign_extend(imm, 8)
        print(f"Instrução: MOV R{rd}, #{imm} (0x{imm:X})")
        return True
    
    def _execute_sub(self, instruction):
        """
        SUB Rd, Rm, Rn (Opcode: 0111)
        Subtrai o valor de Rn do valor de Rm e armazena em Rd.
        Modifica as flags Z e C.
        """
        rd = (instruction >> 8) & 0xF # Bits 11-8 para Rd
        rm = (instruction >> 4) & 0xF # Bits 7-4 para Rm
        rn = instruction & 0xF        # Bits 3-0 para Rn
        
        val_rm = self.registers[rm]
        val_rn = self.registers[rn]
        
        result = val_rm - val_rn
        
        # Garante que o resultado se mantenha em 16 bits (aritmética de complemento a 2)
        result &= 0xFFFF

        self.registers[rd] = result

        # Atualização das flags [cite: 17, 18]
        self.flags['Z'] = 1 if result == 0 else 0
        self.flags['C'] = 1 if val_rn > val_rm else 0 # Carry para subtração sem sinal
        
        print(f"Instrução: SUB R{rd}, R{rm}, R{rn} -> Resultado=0x{result:04X}, Z={self.flags['Z']}, C={self.flags['C']}")
        return True
    
    def _execute_cmp(self, instruction):
        rm = (instruction >> 4) & 0xF
        rn = instruction & 0xF
        
        val_rm = self.registers[rm]
        val_rn = self.registers[rn]
        
        # Atualiza a flag Zero (Z)
        # Z = 1 se os valores forem iguais, senão Z = 0
        self.flags['Z'] = 1 if val_rm == val_rn else 0
        
        # Atualiza a flag Carry (C) para comparação sem sinal
        # C = 1 se Rm < Rn, senão C = 0
        self.flags['C'] = 1 if val_rm < val_rn else 0
        
        print(f"Instrução: CMP R{rm}, R{rn} -> Z={self.flags['Z']}, C={self.flags['C']}")
        return True
    
    def _execute_jmp(self, instruction):
        """
        PC = PC + #Im (Opcode: 0000)
        """
        # O imediato é um valor de 8 bits com sinal
        imm = instruction & 0xFF
        signed_imm = self.sign_extend(imm, 8)
        
        # Atualiza o PC com o imediato
        self.registers[self._simulator.PC] += signed_imm

        print(f"Instrução: JMP #{signed_imm} (0x{imm:X}) -> Novo PC = 0x{self.registers[self._simulator.PC]:04X}")
        return True
    
    
    def _execute_jcond(self, instruction):
        """
        J<cond> #Im (Opcode: 0001)
        Grupo de instruções de salto condicional.
        """
        # --- Lógica de Salto Condicional (J<cond>) ---
        
        # A condição do salto é definida pelos bits 11-10 [cite: 67]
        cond = (instruction >> 10) & 0b11
        
        # O imediato é um valor de 8 bits com sinal
        imm = instruction & 0xFF
        signed_imm = self.sign_extend(imm, 8)
        
        should_jump = False
        jump_name = "UNKNOWN"
        condition_met_str = ""

        # JEQ (Jump if equal): Salta quando a flag zero está ativa [cite: 69]
        if cond == 0b00:
            jump_name = "JEQ"
            if self.flags['Z'] == 1:
                should_jump = True
            condition_met_str = f"Z={self.flags['Z']}"

        # JNE (Jump if not equal): Salta quando a flag zero não está ativa 
        elif cond == 0b01:
            jump_name = "JNE"
            if self.flags['Z'] == 0:
                should_jump = True
            condition_met_str = f"Z={self.flags['Z']}"

        # JLT (Jump if less than): Salta quando a flag zero não está ativa e a carry está 
        elif cond == 0b10:
            jump_name = "JLT"
            if self.flags['Z'] == 0 and self.flags['C'] == 1:
                should_jump = True
            condition_met_str = f"Z={self.flags['Z']}, C={self.flags['C']}"
        
        elif cond == 0b11:
            if self.flags['Z'] == 1 or self.flags['C'] == 0:
                should_jump = True
            condition_met_str = f"Z={self.flags['Z']}, C={self.flags['C']}"
        
        else:
            print(f"AVISO: Salto condicional com código (cond={cond:02b}) desconhecido.")
            return # Retorna para evitar a lógica de impressão abaixo

        # --- Execução do Salto ---
        print(f"Instrução: {jump_name} #{signed_imm} (Condição: {condition_met_str})")
        
        if should_jump:
            # O salto é relativo ao PC, que já foi incrementado
            self.registers[self._simulator.PC] += signed_imm
            print(f"  -> SALTO REALIZADO! Novo PC = 0x{self.registers[self._simulator.PC]:04X}")
        else:
            print("  -> SALTO NÃO REALIZADO.")
    
    def _execute_add(self, instruction):
        """
        ADD Rd, Rm, Rn (Opcode: 0101)
        Soma o valor de Rm com o valor de Rn e armazena em Rd.
        Modifica as flags Z e C.
        """
        rd = (instruction >> 8) & 0xF # Bits 11-8 para Rd
        rm = (instruction >> 4) & 0xF # Bits 7-4 para Rm
        rn = instruction & 0xF        # Bits 3-0 para Rn
        val_rm = self.registers[rm]
        val_rn = self.registers[rn] 
        result = val_rm + val_rn

        # Garante que o resultado se mantenha em 16 bits (aritmética de complemento a 2)
        result &= 0xFFFF
        self.registers[rd] = result
        # Atualização das flags [cite: 17, 18]
        self.flags['Z'] = 1 if result == 0 else 0
        self.flags['C'] = 1 if result < val_rm else 0 # Carry para adição sem sinal
        print(f"Instrução: ADD R{rd}, R{rm}, R{rn} -> Resultado=0x{result:04X}, Z={self.flags['Z']}, C={self.flags['C']}")
        return True
    
    def _execute_addi(self, instruction):
        """
        ADDI Rd, Rm, #Im (Opcode: 0110)
        Soma o valor de Rm com um imediato de 8 bits e armazena em Rd.
        Modifica as flags Z e C.
        """
        rd = (instruction >> 8) & 0xF  # Bits 11-8 para Rd
        rm = (instruction >> 4) & 0xF # Bits 7-4 para Rm
        imm = instruction & 0xFF        
        signed_imm = self.sign_extend(imm, 8)

        val_rm = self.registers[rm]
        result = val_rm + signed_imm
        # Garante que o resultado se mantenha em 16 bits (aritmética de complemento a 2)
        result &= 0xFFFF
        self.registers[rd] = result
        # Atualização das flags [cite: 17, 18]
        self.flags['Z'] = 1 if result == 0 else 0
        self.flags['C'] = 1 if result < val_rm else 0 # Carry para adição sem sinal
        print(f"Instrução: ADDI R{rd}, R{rm}, #{imm} (0x{imm:X}) -> Resultado=0x{result:04X}, Z={self.flags['Z']}, C={self.flags['C']}")
        return True
    
    def _execute_str(self, instruction):
        """
        MEM[Rm] = Rn (Opcode: 0011)
        Armazena o valor de Rn na memória no endereço apontado por Rm.
        """
        # Formato: Opcode(4)|Rn(4)|Rm(4)|Não Usado(4)
        rm = (instruction >> 4) & 0xF # Bits 7-4 para Rm (fonte do endereço)
        rn = instruction & 0xF # Bits 3-0 para Rn (fonte do dado)
        
        # Pega o valor do registrador de dados (Rn)
        data_to_store = self.registers[rn]
        
        # Pega o endereço do registrador de endereço (Rm)
        memory_address = self.registers[rm]
        
        # Armazena o dado na memória
        self.memory[memory_address] = data_to_store
        
        # Rastreia o acesso à memória de dados para o relatório final [cite: 29]
        self.accessed_memory.add(memory_address)

        print(f"Instrução: STR R{rn}, [R{rm}] -> MEM[0x{memory_address:04X}] = 0x{data_to_store:04X}")

    def _execute_subi(self, instruction):
        """
        Rd = Rm - #Im (Opcode: 1000)
        Subtrai um imediato de 8 bits do valor de Rm e armazena em Rd.
        Modifica as flags Z e C.
        """
        rd = (instruction >> 8) & 0xF  # Bits 11-8 para Rd
        rm = (instruction >> 4) & 0xF # Bits 7-4 para Rm
        imm = instruction & 0xFF
        signed_imm = self.sign_extend(imm, 8)
        val_rm = self.registers[rm]
        result = val_rm - signed_imm
        # Garante que o resultado se mantenha em 16 bits (aritmética de complemento a 2)
        result &= 0xFFFF
        self.registers[rd] = result
        # Atualização das flags 
        self.flags['Z'] = 1 if result == 0 else 0
        self.flags['C'] = 1 if signed_imm > val_rm else 0
        print(f"Instrução: SUBI R{rd}, R{rm}, #{imm} (0x{imm:X}) -> Resultado=0x{result:04X}, Z={self.flags['Z']}, C={self.flags['C']}")
        return True
    
    def _execute_and(self, instruction):
        """
        AND Rd, Rm, Rn (Opcode: 1001)
        Realiza a operação AND bit a bit entre Rm e Rn, armazenando o resultado em Rd.
        Modifica as flags Z e C.
        """
        rd = (instruction >> 8) & 0xF  # Bits 11-8 para Rd
        rm = (instruction >> 4) & 0xF # Bits 7-4 para Rm
        rn = instruction & 0xF        # Realiza a operação AND bit a bit
        
        result = self.registers[rm] & self.registers[rn]

        self.registers[rd] = result
        # Atualização das flags [cite: 17, 18]
        self.flags['Z'] = 1 if result == 0 else 0
        self.flags['C'] = 0  # A operação AND não gera carry
        print(f"Instrução: AND R{rd}, R{rm}, R{rn} -> Resultado=0x{result:04X}, Z={self.flags['Z']}, C={self.flags['C']}")
        return True
    
    def _execute_or(self, instruction):
        """
        OR Rd, Rm, Rn (Opcode: 1010)
        Realiza a operação OR bit a bit entre Rm e Rn, armazenando o resultado em Rd.
        Modifica as flags Z e C.
        """
        rd = (instruction >> 8) & 0xF
        rm = (instruction >> 4) & 0xF
        rn = instruction & 0xF

        val_rm = self.registers[rm]
        val_rn = self.registers[rn]
        
        result = val_rm | val_rn
        
        self.registers[rd] = result
        
        # Atualização das flags
        self.flags['Z'] = 1 if result == 0 else 0
        self.flags['C'] = 0 

        print(f"Instrução: OR R{rd}, R{rm}, R{rn} -> Resultado=0x{result:04X}, Z={self.flags['Z']}")
        return True

    def _execute_shr(self, instruction):
        """
        Rd = Rm >> #Im (Opcode: 1011)
        Realiza um deslocamento lógico à direita no valor de Rm e armazena em Rd.
        Modifica as flags Z e C.
        """
        rd = (instruction >> 8) & 0xF  # Bits 11-8 para Rd
        rm = (instruction >> 4) & 0xF # Bits 7-4 para Rm
        shift_amount = instruction & 0xF  # Bits 3-0 para o deslocamento
        val_rm = self.registers[rm]
        result = (val_rm >> shift_amount)
        self.registers[rd] = result
        # Atualização das flags [cite: 17, 18]
        self.flags['Z'] = 1 if result == 0 else 0
        self.flags['C'] = 1 if (val_rm & (1 << (shift_amount - 1))) else 0
        print(f"Instrução: SHR R{rd}, R{rm}, {shift_amount} -> Resultado=0x{result:04X}, Z={self.flags['Z']}, C={self.flags['C']}")
        return True
    
    def _execute_shl(self, instruction):
        """
        Rd = Rm << #Im (Opcode: 1100)
        Realiza um deslocamento lógico à esquerda no valor de Rm e armazena em Rd.
        Modifica as flags Z e C.
        """
        rd = (instruction >> 8) & 0xF  # Bits 11-8 para Rd
        rm = (instruction >> 4) & 0xF # Bits 7-4 para Rm
        shift_amount = instruction & 0xF  # Bits 3-0 para o deslocamento
        val_rm = self.registers[rm]
        result = (val_rm << shift_amount) & 0xFFFF  # Garante que o resultado se mantenha em 16 bits
        self.registers[rd] = result
        # Atualização das flags [cite: 17, 18]
        self.flags['Z'] = 1 if result == 0 else 0
        self.flags['C'] = 1 if (val_rm & (1 << (16 - shift_amount - 1))) else 0
        print(f"Instrução: SHL R{rd}, R{rm}, {shift_amount} -> Resultado=0x{result:04X}, Z={self.flags['Z']}, C={self.flags['C']}")
        return True
    
    def _execute_push(self, instruction):
        """
        SP--; MEM[SP] = Rn (Opcode: 1110)
        Empilha o valor do registrador Rn na pilha.
        Modifica o ponteiro de pilha (SP).
        """
        rn_idx = instruction & 0xF
        data_to_push = self.registers[rn_idx]
        # Decrementa o SP (pilha descendente)
        self.registers[self._simulator.SP] -= 1
        stack_address = self.registers[self._simulator.SP]
        # Armazena o dado na memória
        self.memory[stack_address] = data_to_push
        print(f"Instrução: PUSH R{rn_idx} -> MEM[0x{stack_address:04X}] = 0x{data_to_push:04X}, Novo SP=0x{stack_address:04X}")
        return True
    
    def _execute_pop(self, instruction):
        """
        Rn = MEM[SP]; SP++ (Opcode: 1111)
        Desempilha o valor do topo da pilha para o registrador Rn.
        Modifica o ponteiro de pilha (SP).
        """
        rn = instruction & 0xF
        stack_address = self.registers[self._simulator.SP]
        data_popped = self.memory.get(stack_address, 0)
        # Armazena o dado no registrador
        self.registers[rn] = data_popped
        # Incrementa o SP (pilha descendente)
        self.registers[self._simulator.SP] += 1
        print(f"Instrução: POP R{rn} -> R{rn} = 0x{data_popped:04X}, Novo SP=0x{self.registers[self._simulator.SP]:04X}")
        return True
    
    def _execute_ldr(self, instruction):
        """
        Rd = MEM[Rm] (Opcode: 0010)
        Carrega um valor da memória no registrador Rd, usando o endereço armazenado em Rm.
        """
        rd = (instruction >> 8) & 0xF
        rm = (instruction >> 4) & 0xF
        
        # Pega o endereço do registrador de endereço (Rm)
        memory_address = self.registers[rm]
        
        # Lê o dado da memória. O método .get(addr, 0) garante que,
        # se o endereço não foi inicializado, o valor lido será 0,
        # conforme a especificação[cite: 28].
        loaded_data = self.memory.get(memory_address, 0)
        
        # Armazena o dado lido no registrador de destino (Rd)
        self.registers[rd] = loaded_data
        
        # Rastreia o acesso à memória de dados para o relatório final 
        self.accessed_data_memory.add(memory_address)

        print(f"Instrução: LDR R{rd}, [R{rm}] -> R{rd} = MEM[0x{memory_address:04X}] (Valor=0x{loaded_data:04X})")
        return True
