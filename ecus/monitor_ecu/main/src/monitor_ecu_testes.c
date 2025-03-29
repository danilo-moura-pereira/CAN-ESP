/*
 * main.c - Programa de Testes para diagnosis_module e alert_module
 * Projeto acadêmico para Mestrado Profissional em Engenharia Elétrica - UnB
 * Testes unitários para validação dos módulos de diagnóstico e alerta.
 */

 #include <stdio.h>
 #include "diagnosis_module.h"
 #include "alert_module.h"
 #include "esp_log.h"
 #include "freertos/FreeRTOS.h"
 #include "freertos/task.h"
 
 #define TAG "TEST_MAIN"
 
 // Protótipos das funções de teste
 static void test_diagnosis_module(void);
 static void test_alert_module(void);
 static void simulate_bus_off(void);
 static void simulate_high_bus_load(void);
 static void simulate_high_error_rate(void);
 
 void app_main(void)
 {
     ESP_LOGI(TAG, "Iniciando programa de testes...");
 
     // Inicializa os módulos antes de executar os testes
     if (!diagnosis_module_init()) {
         ESP_LOGE(TAG, "Falha ao inicializar o diagnosis_module!");
         return;
     }
 
     if (!alert_module_init()) {
         ESP_LOGE(TAG, "Falha ao inicializar o alert_module!");
         return;
     }
 
     // Executa os testes
     test_diagnosis_module();
     test_alert_module();
     
     ESP_LOGI(TAG, "Todos os testes foram concluídos!");
 }
 
 // ======================== TESTES DO DIAGNOSIS_MODULE =========================
 
 static void test_diagnosis_module(void)
 {
     ESP_LOGI(TAG, "=== INICIANDO TESTES DO diagnosis_module ===");
 
     DiagnosisData_t diag_data;
 
     // Teste: Atualização dos dados de diagnóstico
     if (diagnosis_module_update(&diag_data)) {
         ESP_LOGI(TAG, "✅ diagnosis_module_update passou.");
     } else {
         ESP_LOGE(TAG, "❌ diagnosis_module_update falhou.");
     }
 
     // Teste: Impressão dos dados de diagnóstico
     ESP_LOGI(TAG, "📝 Exibindo métricas coletadas pelo módulo:");
     diagnosis_module_print(&diag_data);
 
     ESP_LOGI(TAG, "✅ Testes do diagnosis_module concluídos!");
 }
 
 // ======================== TESTES DO ALERT_MODULE =========================
 
 static void test_alert_module(void)
 {
     ESP_LOGI(TAG, "=== INICIANDO TESTES DO alert_module ===");
 
     DiagnosisData_t diag_data;
 
     // Teste: Simular Bus-Off e verificar alerta crítico
     simulate_bus_off();
 
     // Teste: Simular carga do barramento alta
     simulate_high_bus_load();
 
     // Teste: Simular taxa de erro alta
     simulate_high_error_rate();
 
     // Teste: Impressão do histórico de alertas
     ESP_LOGI(TAG, "📝 Exibindo histórico de alertas:");
     alert_module_print_history();
 
     ESP_LOGI(TAG, "✅ Testes do alert_module concluídos!");
 }
 
 // ======================== SIMULAÇÕES PARA TESTES =========================
 
 // Simula uma condição de BUS-OFF e verifica se um alerta é gerado corretamente
 static void simulate_bus_off(void)
 {
     ESP_LOGI(TAG, "🔴 Simulando Bus-Off...");
 
     DiagnosisData_t diag_data;
     diag_data.can_diag.bus_off = true;
 
     alert_module_check_conditions(&diag_data);
 }
 
 // Simula alta carga do barramento CAN e verifica alerta
 static void simulate_high_bus_load(void)
 {
     ESP_LOGI(TAG, "⚠ Simulando alta carga do barramento...");
 
     DiagnosisData_t diag_data;
     diag_data.bus_load = 90; // Simulação de carga excessiva (> 80%)
 
     alert_module_check_conditions(&diag_data);
 }
 
 // Simula taxa de erro alta e verifica alerta
 static void simulate_high_error_rate(void)
 {
     ESP_LOGI(TAG, "⚠ Simulando alta taxa de erro...");
 
     DiagnosisData_t diag_data;
     diag_data.can_diag.tx_error_counter = 120;
 
     alert_module_check_conditions(&diag_data);
 }
 