# CAN-ESP: Rede CAN de Baixo Custo para Veículos Elétricos

![GitHub License](https://img.shields.io/badge/License-LGPLv3-blue.svg)  
![ESP32](https://img.shields.io/badge/Platform-ESP32-green.svg)  
![CAN Bus](https://img.shields.io/badge/Protocol-CAN%202.0B-orange.svg)  

## Visão Geral 
O CAN-ESP é uma implementação de baixo custo de uma rede CAN (Controller Area Network)para veículos elétricos e sistemas embarcados, baseada no microcontrolador ESP32e no protocolo TWAI(compatível com ISO 11898).  

### Principais Recursos 
✔ Baixo custo(utiliza ESP32 em vez de soluções caras como Raspberry Pi).  
✔ Suporte a atualizações OTA (Over-The-Air)via Wi-Fi Mesh.  
✔ Conformidade com ISO 11898(padrão automotivo).  

---

## 🛠️ Hardware Necessário 
| Componente               | Descrição                                                                 |
|--------------------------|---------------------------------------------------------------------------|
| ESP32 WROOM-32      | Microcontrolador com Wi-Fi/Bluetooth e controlador CAN integrado.        |
| SN65HVD230          | Transceptor CAN para interface com o barramento.                         |
| Cabo UTP CAT-5      | Barramento físico (com resistores de terminação de 120 Ω).               |
| Sensores/Atuadores  | Acelerador, freio, motores brushless, display TFT, etc. (veja o artigo). |

---

## Configuração e Uso 
### 1. Instalação 
Clone o repositório e instale as dependências:  
```bash
git clone https://github.com/danilo-moura-pereira/CAN-ESP-LIB.git
```
---

## 📜 Licença 
Este projeto está licenciado sob GNU Lesser General Public License v3.0 (LGPLv3).  
- Você pode: 
  - Usar, modificar e distribuir o código.  
  - Integrar em projetos proprietários, desde que as modificações diretasao CAN-ESP sejam liberadas sob LGPLv3.  
- Você não pode: 
  - Remover a notificação de licença original.  
  - Usar em sistemas que impeçam a modificação do software (ex.: DRM).  

🔗 Texto completo da licença:[LGPLv3](LICENSE.txt).  

---

## Como Contribuir 
1. Faça um fork do projeto.  
2. Crie uma branch(`git checkout -b feature/nova-funcionalidade`).  
3. Envie um pull request(PR).  

---

## Contato 
Autor: Danilo Moura Pereira
E-mail: danilo.mourapereira@gmail.com

