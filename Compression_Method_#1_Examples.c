/**
	This example serves as a LZSS compression implementation. The example code follows the
MISRA guidelines and uses size specific data types that have to be defined regarding the used
hardware target.
	The compression function LZSS_CompressData, read uncompressed data from an input buffer
with byte access and writes compressed data to an output buffer with bit access with the following
functions:
 */
void InputByte(LZSS_OutBuffer_T, Uint8_T byte)
	Reads one byte from a data input buffer.
void OutputBit(LZSS_OutBuffer_T buffer, Uint8_T bitVal)
	Writes one bit with value bitValue to a data output buffer
void OutputBits(LZSS_InBuffer_T buffer, Uint32_T dataValue, Uint8_T numberOfBits)
	Writes numberOfBits with value dataValue to a data output buffer

/*
*-------------------------------------------------------------------------------
* COPYRIGHT (c) Volvo Cars Corporation 2007.
*
* The copyright of the computer program(s) herein is the property of
* Volvo Cars. The program(s) may be copied and
* used only with the written permission from Volvo Cars.
*
**-------------------------------------------------------------------------------
* IDENTIFICATION
*
* Product Name: General ECU
* Module Abbr : Compression
* File Name : lzss_comp.c
* Date : 2007-06-19
* Revision : 1.1
*
*-------------------------------------------------------------------------------
* CONTENTS
*
*1 	DESCRIPTION
*		
*2 	DEFINITIONS
* 		2.1 Constants
*   	2.2 Local functions (forward declarations)
*    	2.3 Variables
*     	2.4 Macros
*
*3	LOCAL FUNCTIONS
* 		3.1 LZSS_InitTree
*	  	3.2 LZSS_ContractNode
*    	3.3 LZSS_ReplaceNode
*     	3.4 LZSS_FindNextNode
*      	3.5 LZSS_DeleteNode
*       3.6 LZSS_AddNode
*
*4 GLOBAL FUNCTIONS
* 		4.1 LZSS_CompressData
*
*-------------------------------------------------------------------------------
*/
/*
*-------------------------------------------------------------------------------
*1 DESCRIPTION
*-------------------------------------------------------------------------------
*
* This file implements the LZSS compression used for the converter tool.
* LZSS is a dictionary compression method and uses a sliding window as
* dictionary. Compression is achieved by replacing byte arrays found in
* previously read data with position/length-pairs pointing in to the sliding
* window. If the position/length-pair takes up more bits than the byte array
* that it tries to compress, the data is left uncompressed. This is indicated
* with a single bit flag, '1' indicates that data to follow is uncompressed and
* '0' indicates that data to follow is a position/length-pair.
* To speed up the compression algorithm a binary search tree is used to store
* previously compressed data.
*
* Code in this module is based on the code examples in the book "The Data
* Compression Book" by Mark Nelson, ISBN 1-55851-434-1
*/


#include <bitio.h> 		/* Data IO functions */
#include <aptypes.h> 	/* MISRA type definitions */
#include <lzss_comp.h>
/*
*-------------------------------------------------------------------------------
*2 DEFINITIONS
*-------------------------------------------------------------------------------
*/

/*
*-------------------------------------------------------------------------------
*2.1 Constants
*-------------------------------------------------------------------------------
*/

/*
* Number of bits allocated to indices into the text window
*/
#define LZSS_INDEX_BIT_COUNT (Uint8_T)10

/**
* @brief Number of bits allocated for the length of an encode phrase
**/
#define LZSS_LENGTH_BIT_COUNT (Uint8_T)4

/**
* @brief Size of the sliding window
**/
#define LZSS_WINDOW_SIZE (Uint16_T)( 1 << LZSS_INDEX_BIT_COUNT )

/*
* Number of bytes to encode a byte array.
* Used to calculate if compression should be done or not
*/
#define LZSS_BREAK_EVEN (Uint8_T)( ( 1 + LZSS_INDEX_BIT_COUNT + \
	LZSS_LENGTH_BIT_COUNT ) / 9 

/*
* End of stream indicator
*/
#define LZSS_END_OF_STREAM (Uint8_T)0

/*
* Size of the look-ahead buffer
*/
#define LZSS_RAW_LOOK_AHEAD_SIZE (Uint8_T)( 1 << LZSS_LENGTH_BIT_COUNT )

/*
* The real look-ahead size, e.g. the maximum number of bytes to match
*/
#define LZSS_LOOK_AHEAD_SIZE (Uint8_T)( LZSS_RAW_LOOK_AHEAD_SIZE + \
LZSS_BREAK_

/*
* Value of the tree root
*/
#define LZSS_TREE_ROOT LZSS_WINDOW_SIZE

/*
* Null index for the tree structure
*/
#define UNUSED (Uint8_T)0
/*
*-------------------------------------------------------------------------------
* 2.2 Local functions (forward declarations)
*-------------------------------------------------------------------------------
*/
void LZSS_InitTree( Uint16_T r );

void LZSS_ContractNode( Uint16_T old_node, Uint16_T new_node );

void LZSS_ReplaceNode( Uint16_T old_node, Uint16_T new_node );

Uint16_T LZSS_FindNextNode( Uint16_T node );

void LZSS_DeleteNode( Uint16_T p );

Uint8_T LZSS_AddNode( Uint16_T new_node, Uint16_T *match_position );


/*
*-------------------------------------------------------------------------------
*2.3 Variables
*-------------------------------------------------------------------------------
*/

/*
* The sliding window used by the decompression algorithm
*/
Uint8_T LZSS_window[ LZSS_WINDOW_SIZE ];

/*
* Binary tree of all of the byte arrays in the window.
*/
struct {
	Uint16_T parent;
	Uint16_T smallChild;
	Uint16_T largeChild;
} tree[ LZSS_WINDOW_SIZE + 1 ];

/*
*-------------------------------------------------------------------------------
*2.4 Macros
*-------------------------------------------------------------------------------
*/

/*
* Arithmetic modulo to get the correct index in the sliding window
*/
#define LZSS_MOD_WINDOW( a ) (Uint16_T)( ( a ) & \
													( LZSS_WINDOW_SIZE - 1

/*
*-------------------------------------------------------------------------------
*3 LOCAL FUNCTIONS
*-------------------------------------------------------------------------------
*/

/*
*-------------------------------------------------------------------------------
* 3.1 LZSS_InitTree
*-------------------------------------------------------------------------------
*
* LZSS Initialize tree
*
* Initialize the binary tree used by the compression to
* keep track of previously compressed byte arrays
*
* param rootChild Root node to the tree
*
* return Void
*
*/
void LZSS_InitTree( Uint16_T rootChild )
{
	/* Add a root child to initialize the tree */
	tree[ LZSS_TREE_ROOT ].parent = UNUSED;
	tree[ LZSS_TREE_ROOT ].smallChild = UNUSED;
	tree[ LZSS_TREE_ROOT ].largeChild = rootChild;
	/* Set child parent to the tree root */
	tree[ rootChild ].parent = LZSS_TREE_ROOT;
	tree[ rootChild ].largeChild = UNUSED;
	tree[ rootChild ].smallChild = UNUSED;
}


/*
*-------------------------------------------------------------------------------
* 3.2 LZSS_ContractNode
*-------------------------------------------------------------------------------
*
* LZSS contract node
*
* Replace a node with another node already in the tree. This function assumes
* that newNode is a child to the old node.
*
* param oldNode Node to remove
* param newNode Node to move to the oldNode position
*
* return Void
*
*/
void LZSS_ContractNode( Uint16_T oldNode, Uint16_T newNode )
{
	/* Move the parent node to the new node */
	tree[ newNode ].parent = tree[ oldNode ].parent;
	/* Change the parent on the child node */
	if ( tree[ tree[ oldNode ].parent ].largeChild == oldNode )
	{
		tree[ tree[ oldNode ].parent ].largeChild = newNode;
	}
	else
	{
		tree[ tree[ oldNode ].parent ].smallChild = newNode;
	}
	/* Detach the old node */
	tree[ oldNode ].parent = UNUSED;
	tree[ oldNode ].smallChild = UNUSED;
	tree[ oldNode ].largeChild = UNUSED;
}

/*
*-------------------------------------------------------------------------------
* 3.3 LZSS_ReplaceNode
*-------------------------------------------------------------------------------
*
* LZSS replace node
*
* Replace a node with a new node that was not previously in the tree
*
* param oldNode Node to remove
* param newNode New node not previously in the tree
*
* return Void
*
*/
void LZSS_ReplaceNode( Uint16_T oldNode, Uint16_T newNode )
{
	/* If the old node is the smallChild*/
	if ( tree[ tree[ oldNode ].parent ].smallChild == oldNode )
	{
		/* Set the new node as smallChild*/
		tree[ tree[ oldNode ].parent ].smallChild = newNode;
	}
	else
	{
		/* Else set the new node as largeChild*/
		tree[ tree[ oldNode ].parent ].largeChild = newNode;
	}
	/* Insert the old node at the new node index in the tree array*/
	tree[ newNode ] = tree[ oldNode ];
	/* Set the small child parent to the new node*/
	tree[ tree[ newNode ].smallChild ].parent = newNode;
	/* Set the large child parent to the new node*/
	tree[ tree[ newNode ].largeChild ].parent = newNode;
	/* Detach the old node */
	tree[ oldNode ].parent = UNUSED;
	tree[ oldNode ].smallChild = UNUSED;
	tree[ oldNode ].largeChild = UNUSED;
}

/*
*-------------------------------------------------------------------------------
* 3.4 LZSS_FindNextNode
*-------------------------------------------------------------------------------
*
* LZSS find node
*
* Find the largest node under the argument node. Used when deleting
* a node. This routine assumes that the argument node has a smaller child.
*
* param node Node to search from
*
* return largest node under the argument node
*
*/
Uint16_T LZSS_FindNextNode( Uint16_T node )
{
	Uint16_T next = UNUSED;

	/* Start at the small child of the node*/
	next = tree[ node ].smallChild;
	/* And find the largest node under the small child*/
	while ( tree[ next ].largeChild != UNUSED )
	{
		next = tree[ next ].largeChild;
	}
	return( next );
}

/*
*-------------------------------------------------------------------------------
* 3.5 LZSS_DeleteNode
*-------------------------------------------------------------------------------
*
* LZSS delete node
*
* Deletes a node from the binary tree.
*
* param node Node to delete
*
* return void
*
*/
void LZSS_DeleteNode(Uint16_T node)
{
	Uint16_T replNode = 0;

	/* If the large child is empty, just move up the small child*/
	if(tree[node].largeChild == UNUSED)
	{
		LZSS_ContractNode( node, tree[ node ].smallChild );
	}
	/* If the small child is empty, just move up the large child */
	elseif ( tree[ node ].smallChild == UNUSED )
	{
		LZSS_ContractNode( node, tree[ node ].largeChild );
	}
	/* Otherwise, replace the node to be deleted with the largest node on
	* the small child side of the tree */
	else
	{
		/* Find replacment node.
		* LZSS_FindNextNode will always get the largest node on the small side
		* and the found node will not have any larger child */
		replNode = LZSS_FindNextNode(node);

		/* Contract the replacement node with its small child */
		LZSS_ContractNode(replNode, tree[replNode].smallChild);

		/* Replace the node to delete with the found node */
		LZSS_ReplaceNode(node, replNode);
	}
}

/*
*-------------------------------------------------------------------------------
* 3.6 LZSS_AddNode
*-------------------------------------------------------------------------------
*
* LZSS add node
*
* In this routine most of the encoder work is done. The routine adds
* a new node to the binary tree. It finds the best match among all existing
* nodes already in the tree and returns the length and the position of the match
* If the new_node has a duplicate in the tree, the old_node is deleted, for
* reasons of efficiency.
*
* param newNode Node to add to the binary tree (pointer to the byte array)
* param matchPos Return pointer to the node position in the window
*
* return length of the match
*
*/
Uint8_T LZSS_AddNode( Uint16_T newNode, Uint16_T *matchPos )
{
	Uint8_T i = 0;
	Uint16_T testNode = 0;
	Int16_T delta = 0;
	Uint8_T matchLen = 0;
	Uint16_T *child;
	BOOL_T matchDone = FALSE;
	Uint8_T retVal = 0;

	/* If the new byte is END_OF_STREAM, exit */
	if ( newNode == LZSS_END_OF_STREAM )
	{
		return( 0 );
	}
	/* Get the first node to test with */
	testNode = tree[ LZSS_TREE_ROOT ].largeChild;
	/* Set the length of the match to 0 */
	matchLen = 0;

	while ( !matchDone )
	{
			i = 0;
		delta = 0;
		/* Look for matches in the tree. Test current test node with the new
		* node until they differ */
		while((i < LZSS_LOOK_AHEAD_SIZE) && (delta == 0))
		{
			delta = LZSS_window[ LZSS_MOD_WINDOW( newNode + (Uint16_T)i ) ] -
					LZSS_window[ LZSS_MOD_WINDOW( testNode + (Uint16_T)i ) ];
			i++;
		}
		/* If the nodes didn't match every LZSS_LOOK_AHEAD_SIZE bytes
		* decrease i to get the current match length */
		if(delta != 0)
		{
			i--;
		}
		/* If i larger than previous match in the tree */
		if ( i >= matchLen )
		{
			/* Set new match length to i */
			matchLen = i;
			/* Set match position to the test node position */
			*matchPos = testNode;
			/* If the byte array in the look-ahead buffer is an exact match of
			* the test node, replace the test node with the new node to remove
			* redundant data */
			if ( matchLen >= LZSS_LOOK_AHEAD_SIZE )
			{
				LZSS_ReplaceNode( testNode, newNode );
				/* Search finished */
				retVal = matchLen;
				matchDone = true;
			}
		}
		if(!matchDone)
		{
			/* If the new node is larger than the test node */
			if ( delta >= 0 )
			{
				/* Continue to test with the larger child */
				child = &tree[ testNode ].largeChild;
			}
			/* If new node is smaller than or equal to the test node */
			else
			{
				/* Continue to test with smaller child */
				child = &tree[ testNode ].smallChild;
			}
			/* If the new test node is a leaf */
			if ( *child == UNUSED )
			{
				/* Add the new node to the tree */
				*child = newNode;
				tree[ newNode ].parent = testNode;
				tree[ newNode ].largeChild = UNUSED;
				tree[ newNode ].smallChild = UNUSED;
				/* Search finished */
				retVal = matchLen;
				matchDone = TRUE;
			}
			/* Continue to test the child*/
			testNode = *child;
		}
	}
	return retVal;
}

/*
*-------------------------------------------------------------------------------
*4 GLOBAL FUNCTIONS
*-------------------------------------------------------------------------------
*/

/*
*-------------------------------------------------------------------------------
* 4.1 LZSS_CompressData
*-------------------------------------------------------------------------------
*
* LZSS compress data
*
* This routine compress data from an in buffer using the LZSS algorithm. LZSS is
* a dictionary compression method where the dictionary is a sliding window. The
* routine uses a look-ahead buffer and finds matches in previously read bytes
* stored in a sliding window.
* If a match is found an index/length pair is written to the output buffer
* (index is the position in the sliding window). If no match is found the read
* byte is just bypassed to the out buffer. To indicate if the data is an
* index/length pair or a plain text byte a single bit is used as indicator flag
* and is written to the out buffer.
* To speed up the compression a binary tree is used for storing previously
* compressed data.
*
* param inBuf Input data buffer with byte access
* param outBufoutput data buffer with bit access
*
* return void
*
*/
void LZSS_CompressData(LZSS_InputBuffer_T *inBuf, LZSS_OutputBuffer_T *outBuf )
{
	/** Index var. for loop */
	Uint8_T i 			= 0;
	/** Byte read from input stream */
	Uint16_T inByte 	= 0;
	/** Number of bytes in the look head buffer */
	Uint8_T aheadBytes 	= 0;
	/** Current position in the window */
	Uint16_T winPos 	= 1;
	/** Number of bytes to replace in the window */
	Uint8_T replCnt 	= 0;
	/** Length of the data match found in the window */
	Uint8_T matchLen 	= 0;
	/** Position in the window of the data match */
	Uint16_T matchPos 	= 0;
	/** Indicator of End Of Stream reached */
	BOOL_T eosReached 	= FALSE;

	/* Start with filling up the look-ahead buffer */
	while( (aheadBytes< LZSS_LOOK_AHEAD_SIZE) && (!eosReached))
	{
		/* Get next input byte */
		inByte = InputByte( inBuf );
		/* If input stream is finished, exit */
		if ( inByte == LZSS_END_OF_INPUT_STREAM )
		{
			eosReached = TRUE;
		}
		else
		{
			/* Add byte to wondow */
			LZSS_window[ winPos + (Uint16_T)aheadBytes ] = (Uint8_T) inByte;
			/* Increase look-ahead bytes*/
			aheadBytes++;
		}
	}
	/* Initialize the tree */
	LZSS_InitTree( winPos );
	/* While there still are bytes in the look ahead buffer, loop */
	while ( aheadBytes > 0 )
	{
		/* If previously match length greater than look ahead bytes it's
		* not possible to code correctly */
		if ( matchLen > aheadBytes )
		{
			/* Set matched length to look-ahead buffer length */
			matchLen = aheadBytes;
		}
		/* If the match is smaller than the compressed data (position/length-
		* pair) there will be negative compression so just output the byte */
		if ( matchLen <= LZSS_BREAK_EVEN )
		{
			/* Set consumed bytes in input stream to 1 */
			replCnt = 1;
			/* Indicate that next byte in output stream is uncompressed
			* by output a '1' */
			OutputBit( outBuf, (Uint8_T)1 );
			/* Output uncompressed byte */
			OutputBits( outBuf,(Uint32_T) LZSS_window[ winPos ], (Uint8_T)8 );
		}
		/* The match is larger than a position/length pair, compression can be
		* done */
		else
		{
			/* Indicate that the following bits are compressed data by output a
			* '0' */
			OutputBit( outBuf, (Uint8_T)0 );
			/* Output position in the window */
			OutputBits( outBuf,(Uint32_T) matchPos, LZSS_INDEX_BIT_COUNT );
			/* Output the length of the match, (length is the number of bytes
			* that is greater than LZSS_BREAK_EVEN) */
			OutputBits( outBuf,(Uint32_T) ( matchLen - ( LZSS_BREAK_EVEN + 1 ) ),LZSS_LENGTH_BIT_COUNT );
			/* Set consumed bytes in input stream to the length of the match */
			replCnt = matchLen;
		}
		/* Delete consumed bytes and add new bytes in the window */
		for ( i = 0 ; i < replCnt ; i++ )
		{
			/* Remove consumed byte from the window */
			LZSS_DeleteNode( LZSS_MOD_WINDOW( winPos + (Uint16_T)LZSS_LOOK_AHEAD_SIZE ) );
			/* Get next input byte */
			inByte = InputByte( inBuf );
			/* If input stream is finished */
			if ( inByte == LZSS_END_OF_INPUT_STREAM )
			{
				/* Decrease the look-ahead bytes */
				aheadBytes--;
			}
			else
			{
				/* Add the new byte from the input stream to the window */
				LZSS_window[ LZSS_MOD_WINDOW( winPos + (Uint16_T)LZSS_LOOK_AHEAD_SIZE ) ]= (Uint8_T) inByte;
			}
			/* Increase the position in the window */
			winPos = LZSS_MOD_WINDOW( winPos + (Uint16_T)1 );
			/* If there still are bytes in the look-ahead buffer */
			if ( aheadBytes != 0 )
			{
				/* Add byte position to the tree and get the length of the
				* match */
				matchLen = LZSS_AddNode( winPos, &matchPos );
			}
		}
	}
	/* Input stream finished, write end of stream to the output buffer
	* uncompressed */
	OutputBit( outBuf, (Uint8_T)0 );
	OutputBits( outBuf, (Uint32_T) LZSS_END_OF_STREAM, LZSS_INDEX_BIT_COUNT );
}




/*
*-------------------------------------------------------------------------------
* End of file
*-------------------------------------------------------------------------------
*/