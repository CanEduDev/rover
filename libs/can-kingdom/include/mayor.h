/*******************************************************************************
 * @file mayor.h
 *
 * Functions to set up a mayor in a CAN Kingdom system.
 *
 * The idea is that the user will define some parameters for their mayor as
 * specified by the ck_mayor_t struct, then the library will initialize its
 * state from these parameters when the user calls ck_mayor_init(). Then,
 * ck_process_kings_letter() can be used to act on commands from the king. The
 * postmaster implementation is responsible for parsing CAN messages and
 * converting them to letters, then the user needs to call
 * ck_process_kings_letter() to process the letter if it's a king's letter.
 *
 ******************************************************************************/

#ifndef CK_MAYOR_H
#define CK_MAYOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include "ck-types.h"

/*******************************************************************************
 * Struct contains pointers to parameters that should be defined by the user.
 * The parameters are used by the mayor library to initialize the library state.
 ******************************************************************************/
typedef struct {
  // The EAN and serial numbers are used to identify the device. They are each
  // 40 bits, but represented as uint64_t. Bit fields could be used, but then
  // it's impossible to detect if the user has provided an invalid number as it
  // would just overflow.

  /// User-provided 40-bit EAN-13 number.
  uint64_t ean_no;

  /// User-provided 40-bit serial number.
  uint64_t serial_no;

  /// User-provided city, group and kingdom identification.
  ck_id_t ck_id;

  /// Skips the startup sequence.
  bool skip_startup;

  /// Function for setting the action mode.
  /// Should return CK_OK on success.
  ck_err_t (*set_action_mode)(ck_action_mode_t);

  /// Function for setting the city mode. City modes are defined by
  /// the mayor. Should return CK_OK on success.
  ck_err_t (*set_city_mode)(ck_city_mode_t);

  /// Function for starting a 200ms one-shot timer. When the timer finishes,
  /// ck_default_letter_timeout() should be called. This should be handled by
  /// the user application. Can remain unset only if #skip_startup is true.
  void (*start_200ms_timer)(void);

  /// Number of folders the mayor has. Must be at least 2 for the king's and
  /// mayor's folders, which are initialized by ck_mayor_init().
  uint8_t folder_count;

  /// Pointer to user-allocated folders. Folder numbers 0 and 1 are reserved by
  /// CAN Kingdom for the king's document and the mayor's document,
  /// respectively.
  ck_folder_t *folders;

  /// Number of lists this mayor has. Must be at least 2, since we need at least
  /// one transmit document list and one receive document list.
  uint8_t list_count;

  /// Pointer to all the mayor's lists, such as document lists, page lists, and
  /// line lists. The mandatory lists are the transmit document list and the
  /// receive document list. Record no 0 in the transmit document list and the
  /// receive document list is reserved by CAN Kingdom and will be set up by
  /// ck_mayor_init().
  ck_list_t *lists;

} ck_mayor_t;

/*******************************************************************************
 * Sets the parameters needed to utilize the mayor library. This function must
 * be called before any other functions in this library can be used.
 *
 * This function does a number of things:
 * - It verifies that the user is setting up the mayor with the required
 *   parameters.
 *
 * - It sets up the internal state of the mayor library.
 *
 * - It defines the mayor's document which is needed to communicate in a CAN
 *   Kingdom system.
 *
 * - It sets up the folders for the king's document and the mayor's
 *   document.
 *
 *   @param mayor parameters for setting up the mayor.
 *
 *   @return #CK_ERR_MISSING_PARAMETER if some required parameter is not set.
 *   @return #CK_ERR_INVALID_PARAMETER if some parameter is set incorrectly.
 *   @return #CK_ERR_ITEM_NOT_FOUND if missing a requred list.
 *   @return #CK_ERR_CAPACITY_REACHED if a requred list is too small.
 *   @return #CK_OK on success.
 ******************************************************************************/
ck_err_t ck_mayor_init(const ck_mayor_t *mayor);

/*******************************************************************************
 * Parse the king's letter and act on it.
 *
 * Not all return codes are listed, since some of the errors may be user
 * defined.
 *
 * @param letter the received king's letter.
 *
 * @return #CK_ERR_NOT_INITIALIZED if #ck_mayor_init() has not been called.
 *
 * @return #CK_ERR_INVALID_KINGS_LETTER if the letter is not a valid king's
 *         letter.
 *
 * @return #CK_ERR_UNSUPPORTED_KINGS_PAGE if the letter contains a king's page
 *         not supported by this implementation.
 *
 * @return #CK_ERR_CAPACITY_REACHED if the king tries to assign more envelopes
 *         than possible.
 *
 * @return #CK_ERR_INVALID_CAN_ID received CAN ID is out of bounds.
 * @return #CK_ERR_INVALID_FOLDER_NUMBER reserved folder number was received.
 * @return #CK_ERR_ITEM_NOT_FOUND folder, record or mayor's page not found.
 * @return #CK_ERR_SEND_FAILED failed to send mayor's page.
 *
 * @return #CK_ERR_INVALID_LIST_TYPE if an accessed list doesn't have a valid
 *         type.
 *
 * @return #CK_ERR_INVALID_PARAMETER some parameters on the king's page are
 *         invalid, or the passed letter is NULL.
 *
 * @return #CK_OK on success or if the letter is not addressed to the caller.
 ******************************************************************************/
ck_err_t ck_process_kings_letter(const ck_letter_t *letter);

/*******************************************************************************
 * Adds a user-defined mayor's page to the mayor's document.
 * The function will store a pointer to the page instead of copying it.
 * Therefore, the page needs to persist in memory.
 *
 * @param page the page to add to the mayor's document.
 *
 * @return #CK_ERR_NOT_INITIALIZED if #ck_mayor_init() has not been called.
 * @return #CK_ERR_INVALID_PARAMETER if page is NULL.
 * @return #CK_ERR_CAPACITY_REACHED if the mayor's document cannot hold more
 *         pages.
 * @return #CK_OK on success.
 ******************************************************************************/
ck_err_t ck_add_mayors_page(ck_page_t *page);

/*******************************************************************************
 * Send the document in the specified folder.
 *
 * Will look for the given folder, check if it's a transmit folder and enabled,
 * then send the document within it. For each enabled envelope in the folder,
 * the document will be sent using that envelope, and for each page in the
 * document a separate letter will be sent. Take for example a folder with 2
 * envelopes assigned to it and enabled (0xA and 0xB), and whose document has 2
 * pages. Then, the letters will be sent in the following order:
 *
 * 1. Letter with envelope 0xA, page 0
 * 2. Letter with envelope 0xA, page 1
 * 3. Letter with envelope 0xB, page 0
 * 4. Letter with envelope 0xB, page 1
 *
 * @param folder_no the requested folder number.
 *
 * @return #CK_ERR_NOT_INITIALIZED if #ck_mayor_init() has not been called.
 * @return #CK_ERR_ITEM_NOT_FOUND if the folder or it's document don't exist.
 * @return #CK_ERR_SEND_FAILED if sending a letter fails.
 * @return #CK_OK on success or if document is not sent due to being a receive
 *         document or in a receive folder.
 ******************************************************************************/
ck_err_t ck_send_document(uint8_t folder_no);

/*******************************************************************************
 * Send the specified page from the document in the specified folder.
 *
 * Will look for the given folder, check if it's a transmit folder and enabled,
 * then send the document within it. For each enabled envelope in the folder,
 * a letter with the specified page will be sent using that envelope.
 * Take for example a folder with 2 envelopes assigned to it and enabled (0xA
 * and 0xB), and whose document has 2 pages. \p page_no is given as 0. The
 * letters will be sent in the following order:
 *
 * 1. Letter with envelope 0xA, page 0
 * 2. Letter with envelope 0xB, page 0
 *
 * @param folder_no the requested folder number.
 * @param page_no the requested page from the folder's document.
 *
 * @return #CK_ERR_NOT_INITIALIZED if #ck_mayor_init() has not been called.
 * @return #CK_ERR_ITEM_NOT_FOUND if the folder or it's document don't exist.
 * @return #CK_ERR_SEND_FAILED if sending a letter fails.
 * @return #CK_OK on success or if document is not sent due to being a receive
 *         document or in a receive folder.
 ******************************************************************************/
ck_err_t ck_send_page(uint8_t folder_no, uint8_t page_no);

/*******************************************************************************
 * Sends the specified mayor's page.
 *
 * @param page_no mayor's page number.
 *
 * @return #CK_ERR_NOT_INITIALIZED if #ck_mayor_init() has not been called.
 * @return #CK_ERR_ITEM_NOT_FOUND if the page doesn't exist.
 * @return #CK_ERR_SEND_FAILED if sending the letter fails.
 * @return #CK_OK on success or if mayor's folder is disabled for some reason.
 ******************************************************************************/
ck_err_t ck_send_mayors_page(uint8_t page_no);

/*******************************************************************************
 * Checks if the envelope is assigned to the king's folder
 *
 * @param envelope envelope to check.
 *
 * @return #CK_ERR_NOT_INITIALIZED if #ck_mayor_init() has not been called.
 * @return #CK_ERR_FALSE if it's not assigned.
 * @return #CK_OK if it's assigned.
 ******************************************************************************/
ck_err_t ck_is_kings_envelope(const ck_envelope_t *envelope);

/*******************************************************************************
 * Tries to find the folder that the given envelope is assigned to.
 *
 * @param envelope envelope to check.
 * @param folder to which the envelope is assigned, if found.
 *
 * @return #CK_ERR_NOT_INITIALIZED if #ck_mayor_init() has not been called.
 * @return #CK_ERR_FALSE if the envelope is not assigned to any folder.
 * @return #CK_OK if it's assigned.
 ******************************************************************************/
ck_err_t ck_get_envelopes_folder(const ck_envelope_t *envelope,
                                 ck_folder_t **folder);

/*******************************************************************************
 * Set the communication mode.
 *
 * @param mode the desired communication mode.
 *
 * @return ck_apply_comm_mode() return codes.
 ******************************************************************************/
ck_err_t ck_set_comm_mode(ck_comm_mode_t mode);

/*******************************************************************************
 * Return the currently set communication mode. Returned value is not valid if
 * mayor is not initialized.
 ******************************************************************************/
ck_comm_mode_t ck_get_comm_mode(void);

/*******************************************************************************
 * Return the last set action mode. Returned value is not valid if mayor is not
 * initialized.
 ******************************************************************************/
ck_action_mode_t ck_get_action_mode(void);

/*******************************************************************************
 * Return the last set city mode. Returned value is not valid if mayor is not
 * initialized.
 ******************************************************************************/
ck_city_mode_t ck_get_city_mode(void);

/*******************************************************************************
 * Returns the current base number. Returned value is not valid if mayor is not
 * initialized.
 ******************************************************************************/
uint32_t ck_get_base_number(void);

/*******************************************************************************
 * Checks if the given letter is the default letter.
 *
 * @param letter the letter to check.
 *
 * @return #CK_ERR_FALSE if it's not the default letter.
 * @return #CK_OK if it is.
 ******************************************************************************/
ck_err_t ck_is_default_letter(ck_letter_t *letter);

/*******************************************************************************
 * Function that should be called when the default letter is received.
 *
 * The city should always call this function when a default letter is received,
 * in order for the mayor library to update its state accordingly.
 *
 * @return ck_set_comm_mode() return codes.
 * @return #CK_ERR_NOT_INITIALIZED if #ck_mayor_init() has not been called.
 * @return #CK_OK on success.
 ******************************************************************************/
ck_err_t ck_default_letter_received(void);

/*******************************************************************************
 * Function that should be called when waiting for a default letter times out.
 *
 * The CAN Kingdom startup sequence specifies that a city should wait at most
 * 200ms for a default letter. Once this timeout is reached, the city should
 * call this function to update the mayor's state.
 *
 * @return ck_set_comm_mode() return codes.
 * @return ck_load_bit_timing() return codes.
 * @return ck_set_bit_timing() return codes.
 * @return #CK_ERR_NOT_INITIALIZED if #ck_mayor_init() has not been called.
 * @return #CK_OK on success.
 ******************************************************************************/
ck_err_t ck_default_letter_timeout(void);

/*******************************************************************************
 * Function that should be called when a CAN message is correctly received.
 *
 * Notifies the mayor library that a correct letter has been received and that
 * the currently set bit timing parameters are compatible with the current
 * system.
 *
 * @return ck_send_mayors_page() return codes.
 * @return ck_save_bit_timing() return codes.
 * @return #CK_ERR_NOT_INITIALIZED if #ck_mayor_init() has not been called.
 * @return #CK_ERR_SET_MODE_FAILED if failed setting the communication mode.
 * @return #CK_OK on success.
 ******************************************************************************/
ck_err_t ck_correct_letter_received(void);

#ifdef __cplusplus
}
#endif

#endif /* CK_MAYOR_H */
